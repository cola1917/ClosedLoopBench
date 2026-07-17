# scene-0061 闭环与评测交接（2026-07-17）

本文档记录 ROS2 真闭环、Docker 算法插件、CARLA 调试可视化、NuRec 六相机回放和参数化评测的当前状态。

## 结论

### 0. ROS2 是否真正控制 CARLA ego

是。`attempt-021-ros2-control-proof` 以 20 Hz 发布 `carla_msgs/msg/CarlaEgoVehicleControl`，真实 CARLA runner 记录：

- 160 帧控制全部接受，fallback 为 0；
- 每帧 throttle 为 0.45；
- ego x 从 -0.4549 m 移动到 12.5460 m；
- 碰撞为 0，cleanup 成功。

正确环境加载顺序：

```bash
source /opt/ros/humble/setup.bash
source /home/cwadmin/sim-env/carla-ros2-ws/install/setup.bash
source /home/cwadmin/sim-env/miniconda3/etc/profile.d/conda.sh
conda activate autodrive
```

此前报错的原因是没有 source CARLA ROS2 workspace；`carla_msgs` 已在 `carla-ros2-ws/install` 中构建。

### 1. 可插拔算法与对比

已构建 ROS2 Humble 镜像 `closed-loop-bench/ego-algorithm:humble`，支持现有 `module:factory` 插件协议，并包含最小 `carla_msgs`。当前接入两个固定油门传输基线，以及两个真正消费 current-tick observation 的 pure-pursuit 路线算法：

- `reference_cruise_035`：throttle 0.35；
- `reference_cruise_055`：throttle 0.55。
- `reference_pure_pursuit_short`：短 lookahead、目标速度 4 m/s；
- `reference_pure_pursuit_long`：长 lookahead、目标速度 5 m/s。

两者都通过 Docker -> ROS2 -> `Ros2ControlDriver` -> `ego.apply_control` 实际控制 CARLA。容器和宿主必须同时设置：

```bash
export ROS_DOMAIN_ID=61
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
```

不强制 UDPv4 时，宿主可以发现 topic，但收不到跨容器样本。

同场景、同 seed、同 160 tick 的结果：

| 算法 | 接受控制 | fallback | Route progress | 碰撞 |
|---|---:|---:|---:|---:|
| reference_cruise_035 | 160 | 0 | 0.06408 | 0 |
| reference_cruise_055 | 157 | 3 个启动帧 | 0.24223 | 0 |

固定油门基线没有转向能力，未达到 95% 路线阈值。新增 pure-pursuit 插件通过 `/closed_loop/ego/observation` 接收 CARLA 当前 ego pose/speed/route，并把 `observation_id` 原样写入控制消息 `header.frame_id`；runner 只接受同 ID 控制。

| 算法 | ticks | 匹配控制 | 平均/最大延迟 | Route progress | fallback | 碰撞 | 结果 |
|---|---:|---:|---:|---:|---:|---:|---|
| pure_pursuit_short | 941 | 941/941 | 1.97/6.54 ms | 0.99205 | 0 | 0 | pass |
| pure_pursuit_long | 681 | 679/681 | 1.99/7.64 ms | 0.99199 | 2 | 0 | pass |

这证明了 Docker 算法的 `current CARLA observation -> algorithm -> frame-matched control -> ego.apply_control` 同回合闭环。它仍不是 TCP/TransFuser 等相机学习模型；真实模型还需要上游仓库、checkpoint、六相机图像 publisher 和依赖镜像。

### 2. CARLA 车道与 bbox 调试显示

`runners/run_carla_basic_agent.py --debug-draw` 现在显示：

- 稀疏白色车道边界与琥珀色虚线中心线；
- 青色 ego bbox；
- 红色其他 actor bbox。

`attempt-026-visual-debug` 跑完 627 ticks，route progress 1.0、碰撞 0、评测通过。VNC framebuffer 原始证据为 `outputs/scene0061_handoff/carla_debug_lane.xwd`。

CARLA 原生场景仍是用于物理闭环的路线道路；大楼和真实街景来自 NuRec 相机流，不会自动出现在 CARLA spectator 中。

### 3. 三相机原因与六相机结果

40k 重建产物实际包含六个录制相机。旧 replay YAML 只列了前左、前中、前右，现已加入后左、后中、后右。

NVIDIA 示例另有 pygame 网格缺陷：尺寸硬编码为 3x2，换行后从第 1 列而非第 0 列开始。六个 NuRec 相机加 overhead sensor 会越界并导致 CARLA tick timeout。`tools/patch_carla_nurec_six_camera_grid.py` 已将行数改为动态计算并修复换行。

最终证据：

- 六路同步序列，每路 577 张 800x450 JPEG；
- 2x3 标注视频：2400x900、30 FPS、577 帧、19.233 秒；
- 视频 SHA-256：`b2adea74eab0b77cbec320efc2a71cdf747da0ad6dce9a73ccca9f5ddad580a9`；
- 中间帧 SHA-256：`ee3f4fbd890a894c39de0be67bd7ed6e2c200246891b7217b587ccd09d52bb59`。

### 4. 参数化与评测覆盖

`examples/scene0061_nurec_v7_experiment_matrix.json` 定义 2 算法 x 1 ODD x 3 seeds，共 6 次。当前 seed 61 有两份报告，覆盖 2/6；seeds 62、63 未跑。

当前路线没有其他 actor，因此 `min_ttc=null`。coverage audit 会正确拒绝把它判为 comparison-ready；必须增加含 actor 的变体后才能验证 TTC/rule metrics。

服务器到期后，本地代码已补齐 vehicle/pedestrian actor runtime：物理 replay、
scripted `WalkerControl`、车辆 TrafficManager、`module:function` 行为插件，以及
reference/runtime track 并行记录。详见
`docs/carla_vehicle_pedestrian_actors.md`。该代码通过本地相关测试，但 scene-0061
的非 ego track 尚未从源数据提取进 `actors[]`，也未在真实 CARLA 上复跑，因此
不能把它写成已完成的 actor 场景评测证据。

## 关键产物

远端根目录：`/home/cwadmin/workspace/ClosedLoopBench/outputs/`

- ROS 真控制：`scene-0061-1000step/runtime/attempt-021-ros2-control-proof/`
- 0.35 插件：`scene-0061-1000step/runtime/attempt-024-reference-cruise-035-udp/`
- 0.55 插件：`scene-0061-1000step/runtime/attempt-025-reference-cruise-055-udp/`
- 横评：`scene-0061-1000step/runtime/reference_cruise_035_vs_055_comparison.json`
- pure-pursuit short：`scene-0061-1000step/runtime/attempt-028-pure-pursuit-short-complete/`
- pure-pursuit long：`scene-0061-1000step/runtime/attempt-029-pure-pursuit-long-complete/`
- 真闭环横评：`scene-0061-1000step/runtime/pure_pursuit_short_vs_long_comparison.json`
- 调试可视化：`scene-0061-1000step/runtime/attempt-026-visual-debug/`
- 参数计划/覆盖：`scene-0061-1000step/runtime/scene0061_nurec_v7_experiment_{plan,coverage}.json`
- 六相机图片：`scene-0061-40k-nurec-replay/images-attempt13-v7-6cam/`
- 六相机视频：`scene0061_40k_six_view_attempt13_v7.mp4`

本地交付：`E:/code/ClosedLoopBench/outputs/scene0061_handoff/`

## 下一任务可直接使用的 prompt

```text
继续 scene-0061 NuRec/CARLA 项目，先阅读：
E:/code/ClosedLoopBench/docs/scene0061_goal_handoff_20260717.md
和 E:/code/ClosedLoopBench/outputs/scene0061_handoff/HANDOFF_20260716.md。

远端 SSH、用户名和私钥由机器本地配置提供，不进入仓库。CARLA 只能通过
远端 workspace 中的 `env_build/start_carla.sh` 启动。

已完成：NuRec 40k、v7 路线对齐、真实 CARLA topology_follower 闭环、ROS2
宿主真控车、两个固定油门传输基线、两个 frame-matched pure-pursuit Docker 算法完整路线横评、简洁车道线+bbox、六相机 2x3 回放。
Docker ROS 控制必须在容器与宿主同时设置 ROS_DOMAIN_ID=61 和
FASTDDS_BUILTIN_TRANSPORTS=UDPv4。

下一步不要重复回放：
1. 当前 ego pose/speed/route 已有 frame-matched observation-control；下一步加入
   current-tick 六相机图像，并沿用 observation_id 记录延迟与丢帧；
2. 接入真实 TCP 或 TransFuser 仓库与 checkpoint，沿用 module:factory 容器接口，
   完成整个 v7 路线；
3. 构造含物理 actor 的 scene-0061 变体，使 TTC/rule metrics 可采样；
   vehicle/pedestrian runtime 代码已存在，重点是提取并对齐真实非 ego track，
   然后在新的 CARLA 服务器上完成物理验证；
4. 跑 seeds 62/63，重新执行 experiment coverage 和 compare_reports；
5. 将算法实时 ego pose/control 接到 NuRec gRPC 相机，证明算法控车与六路照片级
   实时渲染处于同一同步回合，而不是两条独立证据链。

所有成功必须用 report、frame_trace、metrics_trace、cleanup_audit 和视频/图像哈希
证明；preflight 或 topic 可见不能替代真实控车证据。
```

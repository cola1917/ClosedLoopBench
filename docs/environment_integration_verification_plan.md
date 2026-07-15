# 环境接入实施与验收计划

本文定义 ClosedLoopBench 接入真实 CARLA、ROS 2 和外部算法环境后的代码实施顺序、
验证门禁和证据格式。它是 `environment_dependency_backlog.md` 的执行版，不改变
当前结论：离线契约已经具备，真实闭环仍需环境证据。

## 1. 接入目标

首轮环境接入只验证以下核心链路：

```text
不可变 Scene Package
  -> ClosedLoopBench 独占 CARLA 同步 tick
  -> BasicAgent 或一个外部经典算法控制 Ego
  -> scripted / TrafficManager Actor 产生物理响应
  -> 真实 CARLA 数据生成 KPI、报告和算法 x ODD 对比
```

NuRec/Cosmos 新视角、通用场景编辑和复杂 Actor 行为引擎不进入首轮门禁。重建场景
的地图与资产对齐在原生 CARLA Town 闭环稳定后单独验收，不能用 `snap_to_map` 的
成功代替重建对齐证据。

## 2. 不可违反的运行原则

1. ClosedLoopBench 是 `world.tick()` 的唯一拥有者；CARLA ROS Bridge 必须使用
   passive mode，算法容器不得推进仿真。
2. ClosedLoopBench 是 Ego control 的唯一应用者。算法发布带 frame 身份的内部控制
   消息，由 ClosedLoopBench 转换并调用 `ego.apply_control()`；不得同时让 ROS Bridge
   或第二个节点控制同一 Ego。
3. 共享数据盘只交换场景、请求、日志和报告；逐帧 observation/control 使用 ROS 2。
4. 原生 Town smoke、重建场景验收和算法对比是三个不同门禁，证据不得混用。
5. 缺失版本、地图、传感器、KPI、frame 对齐或 cleanup 证据时必须 fail-closed。
6. 任一阶段未通过时停止进入下一阶段；不在失败环境上同时调试多个新变量。

## 3. 目标部署拓扑

```text
Host
  CARLA 0.9.16 :2000
  ClosedLoopBench runner (tick owner + Ego control applicator + KPI)
  CARLA ROS Bridge (passive, observation publisher only)

Algorithm container
  ROS 2 Humble + typed adapter + model plugin + checkpoint
  subscribes observation topic(s)
  publishes /closed_loop/ego/control_cmd

Shared data root
  scenes/<scene-id>/<version>/...
  requests/...
  runs/<run-id>/...
  runtime/...
```

生产基线优先使用 Ubuntu 22.04、CARLA 0.9.16、ROS 2 Humble 和 host network。
Docker Desktop/WSL 的 DDS 行为只能作为额外兼容性验证，不能替代生产基线。

## 4. 代码实施总表

以下项目按顺序实施。标记为“计划新增”的文件当前尚不存在。

| 顺序 | 代码位置 | 实施内容 | 对应门禁 |
| --- | --- | --- | --- |
| 1 | `runtime/carla_probe.py`、`runtime/host_orchestration.py` | 严格校验 client/server/预期版本与计划地图；版本缺失也失败 | G0 |
| 2 | `runtime/environment_manifest.py`、`runners/capture_environment.py`（计划新增） | 采集 OS、Python、CARLA、ScenarioRunner、ROS、RMW、GPU、镜像与代码版本 | G0 |
| 3 | `runtime/tick_lease.py`（计划新增）、`runners/run_host_closed_loop.py` | 原子领取 tick lease，阻止双 runner/双 tick owner | G1 |
| 4 | `runners/run_carla_basic_agent.py` | 修正 frame 时序、严格 spawn、明确终止原因、frame trace 与 cleanup audit | G1-G3 |
| 5 | `metrics/collector.py`、`metrics/report.py` | 保存 frame、传感器可用性、Actor 物理响应和数据来源 | G2-G4 |
| 6 | `agents/ros2_runtime_binding.py`（计划新增） | 真实 typed ROS 2 codec、显式 QoS、相机/状态/路线发布与 stamped control | G5 |
| 7 | `agents/ros2_control_driver.py` | 校验 run/frame、新鲜度和唯一控制权，记录延迟与 safe-stop | G5 |
| 8 | `docker/algorithm/*`、`runners/run_algorithm_container.py` | 加入真实 adapter 运行层、GPU 派生镜像、模型活性计数和完整身份 | G6 |
| 9 | `runners/run_experiment_plan.py`（计划新增） | 逐请求执行、不可覆盖输出、失败隔离、断点续跑和 coverage gate | G7 |
| 10 | `adapters/reconstruction_runtime.py`（计划新增） | 加载重建地图/资产、应用坐标变换并校验对齐误差 | R1 |

每项代码变更先添加无环境的负向/契约测试，再在真实环境增加 integration evidence。
fake 测试只防回归，不用于关闭环境门禁。

## 5. G0：冻结环境与兼容性

### 代码步骤

1. 扩展 `probe_carla()`，同时返回 Python API/client version、server version、当前地图
   完整名称和 endpoint。
2. `validate_host_runtime()` 在预期版本存在但实际版本缺失时失败，并校验
   client/server/expected 三方一致。
3. 将 host plan 中的计划地图传给 probe；禁止使用当前的 `map_name=None` 绕过地图
   校验。
4. 新增环境 manifest，至少记录：
   - OS/kernel、Python 和 ClosedLoopBench commit；
   - CARLA client/server、ScenarioRunner commit 和地图名；
   - ROS distro、RMW、CARLA ROS Bridge commit、`ROS_DOMAIN_ID`；
   - Docker/镜像 digest、GPU/driver/CUDA；
   - 算法 commit、checkpoint SHA-256。

### 检验

```powershell
python runners/run_offline_acceptance.py --output outputs/offline_acceptance.json
python runners/probe_carla.py --host 127.0.0.1 --port 2000
```

连续执行 probe 至少 10 次，不 spawn Actor、不修改 world settings。

### 通过条件

- 离线门禁全绿；client/server/expected 版本完全一致；地图可读取。
- 环境 manifest 字段完整且可重复生成。
- endpoint 不稳定、版本缺失/不一致或地图未知时立即停止。

## 6. G1：建立唯一时钟与生命周期证据

### 代码步骤

1. 在共享数据根的 `runtime/` 下原子领取 tick lease，记录 `run_id`、PID、host、port、
   started_at 和 heartbeat；异常退出后只允许按明确过期规则接管。
2. 第二个 runner 在 spawn 前发现有效 lease 时必须失败。
3. 每帧记录 `world.tick()` 返回 frame、`world.get_snapshot().frame`、仿真时间和 fixed
   delta；frame 必须严格递增 1。
4. TrafficManager 必须成功进入同步模式，异常不得吞掉。
5. finally 阶段按顺序停止 sensor、关闭 autopilot、销毁 Actor、恢复天气/settings、
   关闭 TM 同步并释放 lease。
6. 将每个 cleanup 动作的结果写入 `cleanup_audit.json`；任何残留资源或恢复失败都使
   本次运行失败。

### 通过条件

- 并发启动第二个 runner 时，它在修改 world 前被拒绝。
- 600 个 tick 无重复、跳帧或双 owner 迹象。
- 成功和故意失败后均无残留车辆/传感器，world settings 与运行前一致。

## 7. G2：原生 Town BasicAgent 基线

第一条真实闭环使用原生 Town 和专用短路线 fixture，只验证运行器，不宣称重建场景
已经对齐。

### 代码步骤

1. 为环境 smoke 建立固定 Town、Ego blueprint、spawn、destination、seed 和 fixed
   delta 的 fixture。
2. 增加 strict spawn 模式：验收时只允许计划位置。任意地图 spawn-point fallback
   只能用于显式 debug，并在报告中标记 degraded。
3. spawn 后读取真实 transform，记录计划值、实际值和误差。
4. 统一循环时序：读取当前 snapshot/observation，计算并应用 control，再由唯一 owner
   推进到下一 frame，随后采集该 frame 的物理状态。
5. 只有 `agent.done()` 或物理路线进度达到阈值才成功；耗尽 `max_ticks` 必须返回
   `route_timeout`，不能报告 `ego_closed_loop`。
6. 保存逐帧 control、pose、speed、route progress、终止原因和 sensor 状态。

### 运行入口

```powershell
python runners/run_host_closed_loop.py `
  --exchange-root E:/sim-data `
  --scene-id <native-town-smoke> --version v001 `
  --run-dir E:/sim-data/runs/basic-agent-smoke-001 `
  --carla-map Town04 --execute
```

### 通过条件

- 同一 scene/version/seed 连续三次到达终点，`route_progress >= 0.95`。
- 三次均有真实 collision sensor 样本、严格 frame trace 和完整 cleanup audit。
- `max_ticks` 耗尽、车辆不动、隐式 spawn fallback 或 collision sensor 缺失均失败。

## 8. G3：KPI 数据源与负向场景

### 代码步骤

1. collision 回调必须关联到 CARLA frame；没有 sensor 样本时保持 `unknown`。
2. route progress 只由真实 Ego pose 投影得到，并记录横向投影误差与是否发生回退。
3. acceleration/jerk 由相邻物理 frame 的速度和真实 fixed delta 计算。
4. distance/TTC 使用 Actor 与 Ego 的同 frame 物理状态，记录 closing speed。
5. 报告增加 KPI source/availability 摘要，禁止未知值通过 coverage gate。

### 必跑检验

- 强制碰撞：collision 必须被检测并使准则失败。
- 静止 Ego：tick 增长时 route progress 不得增长。
- 删除 collision sensor：报告必须为 unknown，不能是 0。
- 注入 frame gap：运行必须失败并保存诊断。

## 9. G4：交互 Actor

TrafficManager 和 scripted Actor 分开验收，最后才运行 mixed。

### 代码步骤

1. TrafficManager 绑定 autopilot 只表示 control configured，不表示交互成功。
2. TM evidence 必须包含 seed/sync、生效参数、每帧 pose/speed 和相对 Ego 状态变化。
3. scripted evidence 必须记录 trigger 从 false 到 true、决策、实际 control 以及下一
   frame 的速度/轨迹响应。
4. 只有出现可观察的物理响应后才能报告 `interactive_closed_loop`。

### 通过条件

- 至少一个 TM Actor 和一个 scripted Actor 分别产生可重复的物理响应。
- 只有 control 调用、无运动响应，或 trigger 从未触发时必须失败。
- 固定 seed 三次运行的关键事件顺序一致；数值 KPI 可使用预先冻结的容差。

## 10. G5：ROS 2 typed stub 闭环

真实模型接入前，先用确定性 stub backend 验证 600 tick observation-control 链。

### 协议先决策

- 内部控制 topic 统一为 `/closed_loop/ego/control_cmd`。
- control 必须携带 `run_id`、CARLA observation frame、生成时间和车辆控制值。
- CARLA canonical control topic 不作为算法输出，避免双控制消费者。
- fallback 统一为 full-brake safe-stop，不再描述为 BasicAgent fallback。

### 现场冻结 ROS 图

```bash
ros2 topic list -t
ros2 topic info -v /carla/ego_vehicle/rgb_front/image
ros2 topic info -v /carla/ego_vehicle/odometry
ros2 topic info -v /carla/ego_vehicle/waypoints
ros2 interface show carla_msgs/msg/CarlaEgoVehicleControl
```

确认 role name、类型、publisher/subscriber 数量、QoS、`use_sim_time`、RMW 和
passive mode 后再写最终 binding。

### 代码步骤

1. spawn 真实 RGB sensor，使用实际 blueprint 属性和 transform 生成 CameraInfo、内参
   和 sensor-to-ego 外参。
2. 使用 `world.tick()` 的真实 frame 和 snapshot 时间，将 camera、Ego state、route
   组成同 frame observation；缺任一通道时 safe-stop。
3. 新增 production `rclpy.Node`/codec，绑定真实 `sensor_msgs/Image`、CameraInfo、
   odometry/route 和 stamped control；保留 `TickObservationAggregator` 为纯逻辑核心。
4. 使用显式 QoS profile，不再用整数 `10` 代表生产 QoS。
5. `Ros2ControlDriver` 拒绝错误 run、重复/未来/过旧 frame 和非法 control，记录
   observation-to-control latency。

### 通过条件

- 连续 600 tick 无 mixed frame；observation/control 计数持续增长。
- nominal stub 运行无超时；主动停止 stub 后，在 `control_timeout_sec + 1 frame`
  内 full brake。
- ROS bag 回放能复现 same-tick ready 和缺帧 fail-closed 行为。
- 双 Ego controller、frame 无法关联或 QoS 持续不兼容时立即停止。

## 11. G6：接入一个真实算法

首选 TCP 或 TransFuser 中与 CARLA 0.9.16 兼容性最清楚的一个；不同时接两个新模型。

### 代码步骤

1. 冻结算法 commit、依赖、输入分辨率/相机位姿、route command、checkpoint 和 license。
2. 在外部算法仓库实现现有 `create_backend(config)` 插件边界；模型代码和权重不复制
   进 ClosedLoopBench。
3. 构建 GPU 派生镜像，包含 typed adapter/aggregator，或明确由外部插件实现完全相同
   的 stamped 协议。
4. preflight 校验 GPU、模型加载、输入 shape、checkpoint SHA-256 和一次确定性推理。
5. ready/health 由主推理进程更新，加入 last observation frame、last control frame、
   observation/control counts；Docker health 不得自行伪造活性。
6. host runner 校验算法 ID/version、镜像 digest、checkpoint digest 和 heartbeat 后才
   spawn Ego。

### 通过条件

- 与 stub 使用相同 observation/control 契约完成固定路线。
- 报告记录算法 commit、镜像 digest、checkpoint digest、延迟和 fallback 次数。
- health ready 但计数不增长、预处理/坐标系未确认或 nominal run 出现 stale control
  时失败。

## 12. G7：算法 x ODD x seed 矩阵

单次 BasicAgent、Actor、ROS stub 和真实算法全部通过后，才执行矩阵。

### 代码步骤

1. 用真实 Scene Package digest 和算法版本更新
   `examples/core_experiment_matrix.v0.example.json`。
2. 计划新增的 batch runner 逐个消费 `evaluation_run_request.v1`，每个 run 使用不可
   覆盖目录，失败后保存证据并继续/停止行为由显式策略决定。
3. 每次运行生成 report、metrics trace、runtime log 和
   `evaluation_run_result.v1`；environment manifest 与 cleanup audit 可作为 JSON
   `runtime_log` artifact 发布。
4. coverage gate 通过后才调用报告比较，不允许先忽略缺失运行求平均值。

### 最小首轮矩阵

```text
1 scene x 2 algorithms (BasicAgent + learned) x 2 ODD x 3 seeds = 12 runs
```

### 通过条件

- `ready_for_comparison=true`。
- 无 missing、duplicate、malformed、unknown KPI 或伪 interactive run。
- 相同输入重跑不覆盖原证据；所有结果可追溯到环境和模型身份。

## 13. R1：重建场景运行验收

该门禁验证 TriggerEngine -> NeuralSceneBridge -> ClosedLoopBench 的完整流水线；它在
原生 Town 运行器稳定后进行。

### 代码与数据步骤

1. Scene Package 必须给出可加载地图/道路、资产引用、坐标系、单位、yaw 约定、
   `sim_from_log_transform` 和允许的视觉/几何有效域。
2. 实现 reconstruction runtime adapter，将包内地图/资产注册到目标 CARLA/NuRec
   环境；仅把路径写入报告不算加载成功。
3. 使用固定地标和轨迹关键点计算平移、航向和高度误差；保存对齐表与俯视证据。
4. 验收模式禁止任意 spawn fallback；`snap_to_map` 前后误差必须记录并低于冻结阈值。
5. 新视角尚未承诺时，在报告中标注 renderer validity domain；Ego 超出有效域不得把
   视觉结果解释为可信重建输出。

### 通过条件

- `pending_runtime_alignment` 已被真实对齐证据替代。
- Ego/Actor 初始位置、道路拓扑和动态轨迹在同一坐标系内。
- 完整场景至少完成一次 BasicAgent 和一次交互 Actor 运行。

## 14. 统一证据目录

每个正式 run 至少保留：

```text
runs/<run-id>/
  evaluation_request.json
  environment_manifest.json
  host_run_plan.json
  readiness.json
  carla_run_config.json
  frame_trace.jsonl
  metrics_trace.jsonl
  cleanup_audit.json
  closed_loop_report.json
  evaluation_result.json
  logs/
    carla.log
    ros_bridge.log
    algorithm.log
    runner.log
```

截图只能作为辅助材料。最终通过依据必须是机器可读的 plan、trace、audit、report 和
result artifact。

## 15. 并行安排与依赖

环境到位后可并行开展：

- **Host 线：** G0 -> G1 -> G2 -> G3 -> G4。
- **ROS/算法线：** G0 后冻结 ROS 图、选择模型和准备镜像；G2 三连跑通过后执行
  G5/G6 端到端联调。
- **重建线：** G0 后准备地图导入和坐标变换；G2 稳定后执行 R1。
- **评测线：** 提前准备 evidence schema 和 batch runner；G4/G6 通过后执行 G7。

不可并行跳过的硬依赖为：

```text
G0 -> G1 -> G2 -> G3 -> G4
                  -> G5 -> G6 -> G7
                  -> R1
```

## 16. 第一轮环境接入的停止点

出现以下任一情况，当天停止扩展范围，只修复当前层：

- client/server/预期 CARLA 版本或地图不一致；
- 双 tick owner、双 Ego controller、frame 重复/跳变；
- 隐式 spawn fallback、地图/重建对齐未知；
- collision sensor、route progress 或核心 KPI 来源不可信；
- Actor 只有 control 调用而没有物理响应；
- ROS topic/type/QoS 依赖猜测，或 observation/control 无 frame 关联；
- 算法 timeout 未触发 full brake；
- health ready 但 observation/control 计数不增长；
- cleanup 不完整或正式证据目录缺项。

只有 G0-G7 和 R1 的对应门禁按目标范围通过后，才能宣称三项目真实完整闭环完成。

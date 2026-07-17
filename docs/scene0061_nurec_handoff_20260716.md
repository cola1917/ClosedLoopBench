# scene-0061 NuRec / CARLA 交接（2026-07-16）

## 目标与边界

本轮目标是先解决 NuRec gRPC 获取与加速，再把 scene-0061 的 40k 重建产物接入
CARLA 做可视联动；只有视觉链路、道路对齐和运行稳定性都通过后，才进入算法控车与
正式 metrics。

当前结论：NuRec 40k 可视联动已经完整跑通并可复现；算法控车和正式 metrics 尚未
开始。当前阻断项不是 NuRec，而是有限版 nuScenes-to-OpenDRIVE 生成器产出的
`road.xodr` 几何质量不足。

## 三条链路状态

### 1. 环境与 NuRec gRPC：完成

- 远端连接信息由机器本地配置提供，不进入仓库。
- CARLA：`/home/cwadmin/sim-env/data/CARLA_0.9.16`
- Python：`/home/cwadmin/sim-env/miniconda3/envs/autodrive/bin/python`
- CARLA 必须通过 `/home/cwadmin/workspace/env_build/start_carla.sh` 启动。
- 新增 `env_build/stage6_nurec_grpc.sh`；远端连续执行两次幂等，
  `bash verify.sh 6` 为 **10 passed / 0 failed / 0 warnings**。
- 公开 CARLA gRPC 镜像通过华为 SWR 加速并固定官方 digest：
  - `docker.io/carlasimulator/nvidia-nurec-grpc:0.2.0`
  - `sha256:a83c6e45b79b414905e8d19fb120f14f4fb5658e70ddfdc89553aeb6e007c668`
- 两个需授权的 NVIDIA NGC 镜像不硬切匿名国内镜像：已安装则复用；可配置私有国内
  镜像，但只有 digest 与固定值一致才接受，否则回退官方 NGC。
  - `nvcr.io/nvidia/nre/nre-ga:26.04`
    `sha256:1c3a838440fce96258e055615b146fa8f75ad2a799e131aaa480755592317d1f`
  - `nvcr.io/nvidia/nre/nre-tools-ga:26.04`
    `sha256:d9c19b67a86c7fca71c2c23250a953870e616c562325fb27131d004c7f8bef1c`
- `0.2.0` gRPC 服务不能加载 26.04 产物（`free-pose-calib`、
  `appearance_embedding` 版本不兼容）；正式回放必须使用
  `nre-ga:26.04 serve-grpc`。

### 2. 40k 重建与可视回放：完成

- 正式 USDZ：
  `/home/cwadmin/workspace/NeuralSceneBridge/outputs/nurec_formal_scene0061_6cam_40k/nB3fGDTuUz5ptMbZzCXnjS/artifacts/last.usdz`
- scene-0061、6 个 1600×900 pinhole 摄像头、384 个 ego pose，记录时长
  `19.228646 s`。
- 第十次最终回放使用原始 40k USDZ、`nre-ga:26.04 serve-grpc`、外部 XODR，
  未启用算法控制。
- 三路 NuRec 摄像头各 577 帧、800×450、30 FPS；总计 1731 JPEG，模拟时长
  `19.233333 s`。
- 进程退出码 0，日志无 traceback、segfault、core dump 或此前的
  `terminate called without an active exception`。
- 清理补丁会移除 CARLA world tick callbacks，并在解释器退出前关闭 gRPC channel
  与 nvImageCodec native/GPU 对象。
- `nvTIFF`、`nvJPEG2000` 缺失警告对本次 JPEG RGB 路径无影响，不需要为了本次结果
  安装。
- CARLA 原生窗口只显示 OpenDRIVE 道路和代理 actor；建筑和真实街景位于独立
  `NUREC Camera View` / 保存的 NuRec RGB 中。USDZ 是神经表示，不是 UE 静态 mesh。

本地交付：

- `outputs/scene0061_40k_three_view.local.mp4`（24,525,237 bytes）
- SHA-256：`fb9270bf0555ba66fa16bb367ac9608d2a1c44008b88693750f1d8d173d39db7`
- `outputs/scene0061_40k_front_left_mid.jpg`
- `outputs/scene0061_40k_front_mid.jpg`
- `outputs/scene0061_40k_front_right_mid.jpg`
- `outputs/scene0061_handoff/replay_attempt10_all_obb.log`
- `outputs/scene0061_handoff/attempt10_verification.txt`
- `outputs/scene0061_handoff/delivery_sha256.txt`

### 3. 项目 3（ClosedLoopBench）接入：部分完成，有明确质量门

已完成：

- Reconstruction Package / planning 接口、配置与 checkpoint 校验代码已存在；相关
  单元测试在远端 `autodrive` 环境运行：`6 tests OK (1 skipped)`。
- CARLA 状态驱动 NuRec gRPC 渲染的视觉联动成立；这证明传感器视觉链路，不等同于
  算法闭环成绩。
- 第十次回放对所有车辆/行人逐帧执行 2D SAT + Z 轴 OBB 穿插检查，
  `sample_count=0`。因此 CARLA 画面中看似碰撞的现象不是 actor 车体重叠。
- 原生 CARLA collision sensor 不适用于当前禁用物理、逐帧设置 pose 的运动学回放；
  挂载后第一次同步 tick 会卡住，不能把它的零事件当作无碰撞证据。该方案已从 Stage 6
  移除，保留 OBB 诊断。

未完成 / 禁止提前宣称：

- scene-0061 尚未进行算法控车。
- 尚未产出正式 collision、route progress、TTC、comfort、rule violation metrics。
- 当前 XODR 不允许进入上述阶段。

## XODR 质量结论

当前文件：
`/home/cwadmin/workspace/ClosedLoopBench/outputs/scene-0061-1000step/road.xodr`

`adapters/nuscenes_map_to_opendrive.py` 明确是有限转换器：每个 nuScenes lane polygon
单独变成一条单车道 road；几何为分段直线；只有无歧义的端到端连接；没有 junction、
交通灯、停止线、路口拓扑、高程或超高。这解释了 CARLA 原生窗口中的粗糙路面和错位。

384 个记录 ego pose 对当前 CARLA waypoint 的实测：

- waypoint 可投影：384/384
- 位于最近车道半宽范围：`48.9583%`
- 车道中心水平误差：P50 `2.7295 m`，P95 `4.2749 m`，max `8.9097 m`
- 高程误差：P50 `0.8249 m`，P95 `1.4355 m`
- 航向误差：P50 `5.3209°`，P95 `10.3530°`，存在一次接近反向的极值
- CARLA 生成图没有 junction pose

证据：`outputs/scene0061_handoff/xodr_alignment_attempt10.json`。

因此 XODR 只能作为可视联调占位图。建议进入算法控车前至少满足：

- ego pose 位于车道半宽内的比例 `>= 95%`
- 车道中心误差 P95 `<= 1.0 m`
- 航向误差 P95 `<= 5°`
- 路线沿途无断链，并正确表达关键路口连接
- 再次完整 NuRec 回放，OBB 穿插仍为 0

## 可复现命令

```bash
cd /home/cwadmin/workspace/env_build
bash stage6_nurec_grpc.sh
bash verify.sh 6
bash start_carla.sh
```

另一个终端：

```bash
cd /home/cwadmin/workspace/ClosedLoopBench
NUREC_IMAGE=nvcr.io/nvidia/nre/nre-ga:26.04 \
NUREC_IMAGE_COMMAND=serve-grpc \
OUTPUT_DIR=/path/to/new/images \
LOG_FILE=/path/to/new/replay.log \
OVERLAP_LOG=/path/to/new/overlap.json \
bash tools/run_scene0061_nurec_replay.sh
```

## 代码同步与验证

- 五个仓库已检查：`env_build`、`ClosedLoopBench`、`NeuralSceneBridge`、
  `SceneExchangeContracts`、`TriggerEngine`。
- 后两者远端、本地均干净。
- 远端三个脏仓库的不含缓存/密钥/输出的代码快照已回传：
  `outputs/scene0061_handoff/remote_code_snapshot.tar.gz`。
- 快照 35 个文件逐文件按 LF 归一化比较：34 个初次一致；唯一差异
  `ClosedLoopBench/tools/remote_start_carla.sh` 以本地更严格版本为准，并已同步远端；
  最终本地/远端 SHA-256 同为
  `d6a5ae24159c90a6f745cae25487ae5f5f6499972de8d9b2735bf2f5ef1acd71`。
- 远端 `.cache/` 与机器本地 `config/nurec-formal.env` 未回传；前者不是源码，后者可能
  包含机器配置，不应进入 Git。
- 快照为审计留档，包含已失败的历史实验
  `patch_carla_nurec_collision_diagnostics.py`；该文件已从当前本地和远端源码树删除，
  不得恢复或使用。当前正式诊断器是 `patch_carla_nurec_overlap_diagnostics.py`。
- 本地 57 个 `tools/adapters/runners` Python 文件 AST 检查通过。
- 相关单元测试：`Ran 6 tests ... OK (skipped=1)`。

## 下一任务交接 Prompt

```text
继续 scene-0061 NuRec/CARLA 工作。先阅读：
E:/code/ClosedLoopBench/docs/scene0061_nurec_handoff_20260716.md

不要重新下载 NuRec 镜像，不要重训 40k，不要启用算法控车或正式 metrics。
远端连接信息由机器本地配置提供；CARLA 必须使用远端 workspace 中的
`env_build/start_carla.sh` 启动。

当前任务只修复/重建 scene-0061 OpenDRIVE：
1. 以当前 road.xodr 和 xodr_alignment_attempt10.json 为基线；
2. 检查 nuScenes lane polygon -> centerline、local frame、laneOffset、方向、连接与高程；
3. 生成版本化的新 XODR，绝不覆盖旧基线；
4. 在 CARLA 中对 384 个记录 ego pose 重新跑 audit_nurec_xodr_alignment.py；
5. 目标：inside_lane >= 95%、centerline P95 <= 1.0m、heading P95 <= 5°、
   路线无断链；
6. 达标后再完整跑一次 40k NuRec 三摄像头回放，要求每路 577 帧、退出码 0、
   OBB overlap sample_count=0；
7. 只有上述门通过后，才提出算法控车和 metrics 的实施方案。

保留并报告所有新产物、日志、哈希；所有代码先改本地 E:/code，再同步远端，禁止只在
远端手改源码。
```

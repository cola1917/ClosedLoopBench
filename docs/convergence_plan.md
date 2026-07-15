# ClosedLoopBench 收敛计划

## 1. 最终愿景

ClosedLoopBench 的核心目标是建立稳定、可复现的自动驾驶闭环评测链路：

```text
TriggerEngine 选择 scene_id
  -> NeuralSceneBridge 重建并发布版本化 Scene Package
  -> ClosedLoopBench 驱动本机 CARLA 同步仿真
  -> 算法通过 ROS 2 控制 Ego
  -> TrafficManager 或 scripted Actor 对交通状态作出反应
  -> 输出可信 KPI、运行证据和算法/ODD 对比报告
```

项目同时支持端到端算法和三段式算法。首批基线采用 CARLA BasicAgent 与
一个外部经典算法（TCP 或 TransFuser），算法仓库、模型权重和推理环境由
外部容器负责，ClosedLoopBench 只维护稳定的传感器、路线、控制和报告契约。

Cosmos 可在后续作为同一场景的 ODD 视觉变体或 fixer 接入，但不属于首版
闭环完成条件。NuRec 任意新视角渲染、通用场景编辑平台和自研大规模 Actor
行为引擎明确不在当前范围内。

## 2. 当前事实（2026-07-14）

### 已完成的离线实现

- nuScenes 整场景可提取为规范化 Scene IR。
- 可生成 `scene_ir.json`、`road.xodr`、`scenario.xosc` 和
  `scene_package.json`，包内引用使用相对路径。
- replay Actor 可编译为带时间戳的完整 OpenSCENARIO 轨迹，Ego 保留给
  闭环算法控制。
- 已实现有限范围的 nuScenes Map 到 OpenDRIVE 转换；复杂路口、信号灯、
  高程和分支拓扑仍属于明确边界。
- 本地 `scene-0061` 生成物已通过 esmini headless 验证。
- BasicAgent 运行循环、ROS 2 控制驱动、超时 safe-stop、Actor 执行证据、
  碰撞接口、路线进度、加速度、jerk 和报告比较均已完成离线实现及 fake
  runtime/契约测试。
- 共享数据盘已具备版本化目录、原子发布、READY 标记、manifest 校验和
  安全消费协议，能够避免读取半成品。
- 本地代码已达到等待 CARLA、ROS 2 和算法环境接入的状态。

### 尚未完成的环境验收

当前机器没有可用于验收的 CARLA/ROS 2/真实算法运行环境。因此以下结论
**尚不能声称已验证**：

- 真实 CARLA 0.9.16 中的地图加载、车辆生成、同步 tick 和资源清理。
- BasicAgent 连续三次完成固定路线。
- TrafficManager/scripted Actor 在物理轨迹上确实产生交互反应。
- 真实碰撞回调、路线进度、TTC、加速度和 jerk 的采集可信度。
- ROS 2 Humble、CARLA ROS Bridge、真实相机标定和 typed topic 的联通。
- TCP 或 TransFuser checkpoint 在 GPU 容器中的真实推理与控车。
- BasicAgent 与学习算法在多 seed、不同 ODD 下的完整对比矩阵。

所有必须依赖外部环境的任务和验收证据统一记录在
[`environment_dependency_backlog.md`](environment_dependency_backlog.md)。
fake runtime 测试只能证明代码契约，不能替代该清单中的真实环境证据。

## 3. 收敛原则

1. ClosedLoopBench 是 CARLA 同步时钟的唯一拥有者；算法容器不得自行调用
   `world.tick()`。
2. 共享数据盘只负责离线场景和结果交付，逐帧观测与控制使用 ROS 2 或网络
   通道。
3. OpenSCENARIO 是首选动态场景交换格式，OpenDRIVE 承载静态道路；
   ScenarioRunner Python 逻辑只补充标准无法表达的行为。
4. BasicAgent 是首条传统基线；首版只需再接入 TCP 或 TransFuser 中的一个，
   不在本项目重新实现模型。
5. 本地测试不得依赖 CARLA、ROS 2、GPU、网络或外部模型仓库。
6. “代码已实现”“fake runtime 已验证”和“真实环境已验收”必须明确区分。
7. 新需求只有在阻塞 C0-C3 的退出条件时才进入当前收敛范围。

## 4. 分阶段计划

### C0：冻结交付契约

**目标：** 固定三个项目之间以及运行时内部的输入、输出和状态语义。

**范围：**

- 冻结 Scenario IR、Scene Package、CARLA run config 和闭环报告 Schema。
- 明确 replay、scripted、TrafficManager Actor 的能力等级。
- 固定 collision、TTC、distance、route progress、hard brake、jerk 等核心 KPI
  的来源和缺失值语义。
- 统一 unsupported、lossy export、环境缺失和运行失败的结构化诊断。
- 冻结共享盘版本目录、原子发布和 READY 消费规则。

**当前状态：** 离线契约与消费协议已完成；兄弟项目的薄发布/领取适配器仍需
在对应项目中落地。

**DoD：** Schema、文档、示例和测试术语一致；非法状态、缺失输入、路径穿越
和未完成发布均有负向测试。

### C1：收敛本地离线流水线

**目标：** 无 CARLA 环境也能稳定构建和验证场景交换产物。

**范围：**

- `nuScenes -> Scene IR -> OpenDRIVE/OpenSCENARIO -> Scene Package`。
- esmini headless 验证、dry-run 报告、契约测试和回归测试。
- 固定示例的关键字段、退出码、相对路径及 Windows/Linux 路径行为。
- 生成物与需要版本控制的样例严格分离。

**当前状态：** 已完成。`scene-0061` 已形成可执行的本地验证证据；这不等于
CARLA 兼容性已经验证。

**DoD：** 固定 nuScenes 场景可重复生成合法四件套；本地测试不依赖外部重型
环境；失败通过非零退出码和明确诊断暴露。

### C2：验收 CARLA BasicAgent 闭环

**目标：** 将已经完成的离线运行实现转换为第一条真实仿真证据。

**范围：**

- 锁定 CARLA 0.9.16、Python API、ScenarioRunner、地图和操作系统版本。
- 验证 probe、地图对齐、Ego/Actor spawn、同步模式、BasicAgent 控制、tick、
  传感器销毁和 world settings 恢复。
- 验证真实 collision、route progress、distance/TTC、加速度和 jerk 数据来源。
- 固化一个短时 happy path 和至少一个结构化失败场景。

**当前状态：** 代码和 fake runtime 测试已完成；真实 CARLA 验收未开始，详见
环境依赖清单。

**DoD：** 固定场景连续运行至少三次且无遗留 Actor、传感器或 world settings；
报告中的 tick、进度与核心 KPI 均来自真实 CARLA；失败路径可靠清理并返回
结构化原因。

### C3：验收交互 Actor

**目标：** 证明 Actor 的闭环等级来自真实执行，而不是配置标签。

**范围：**

- replay 只作为半闭环参考，不宣称会响应 Ego。
- 至少一个 scripted Actor 根据触发条件执行让行、制动或继续行驶。
- 至少一个 TrafficManager Actor 使用固定 seed 和行为参数运行。
- 在 trace/report 中保存 Actor 决策、控制调用和物理运动证据。

**当前状态：** scripted/TrafficManager 控制路径及执行证据采集已离线实现；
真实 CARLA 中的物理响应尚未验收。

**DoD：** 至少一个 Actor 对 Ego/交通状态产生可观察、可重复的物理轨迹变化；
报告声明的闭环等级与实际执行器一致；fallback 和已知限制有记录。

### C4：验收外部 Ego 算法

**目标：** 用稳定插件边界接入经典算法，不把模型实现合并进本仓库。

**范围：**

- 完成 observation-control 往返、超时、非法输出和 safe-stop 契约。
- 使用 ROS 2 Humble 和 CARLA ROS Bridge 验证真实相机、里程计、路线和控制
  topic，同时保持 ClosedLoopBench 为同步时钟拥有者。
- 选择 TCP 或 TransFuser 的一个兼容版本，构建 GPU 容器并加载 checkpoint。
- 记录算法 commit、镜像 tag、checkpoint digest、传感器配置、延迟和 fallback。

**当前状态：** 通用 ROS 2/算法适配骨架和 stub 验收已完成；真实 ROS 2、GPU
和模型 smoke 未完成。

**DoD：** BasicAgent 与一个学习算法可消费相同 Scene Package 并输出同一报告
契约；超时或断连时 Ego 安全停车；真实模型结果具有完整版本身份。

### C5：发布候选与归档

**目标：** 形成可复核、可演示、可继续集成的稳定版本。

**范围：**

- 运行完整本地门禁和环境门禁，保存命令、版本、输入摘要、日志与报告。
- 完成 README、架构、运行手册、故障排查、已知限制和许可证审计。
- 固定一个主场景、算法矩阵、ODD 矩阵、seed 集合和报告聚合方式。
- 冻结 Schema 与镜像/代码版本，归档可追溯证据。

**当前状态：** 本地发布材料可继续整理；最终候选被 C2-C4 的环境证据阻塞。

**DoD：** 新环境可按文档复现本地构建和真实 BasicAgent smoke；所有实验满足
覆盖门禁；未完成项均进入明确 backlog，不以模糊 TODO 混入发布范围。

## 5. 优先级与执行顺序

| 优先级 | 内容 | 当前状态 | 是否阻塞最终收敛 |
| --- | --- | --- | --- |
| P0 | C0 契约冻结、C1 离线流水线 | 离线实现完成 | 否 |
| P0 | C2 真实 BasicAgent happy path | 等待 CARLA 环境 | 是 |
| P1 | C2 真实 KPI 与失败清理 | 等待 CARLA 环境 | 是 |
| P1 | C3 真实交互 Actor | 等待 CARLA 环境 | 是 |
| P1 | C4 ROS 2 通道和一个经典算法 | 等待 ROS 2/GPU/模型 | 是 |
| P2 | BasicAgent/学习算法跨 ODD、跨 seed 对比 | 等待前述环境 | 是 |
| P3 | Cosmos ODD 视觉变体/fixer | 后续可选 | 否 |

主顺序为 `C0 -> C1 -> C2 -> C3 -> C4 -> C5`。算法容器准备可以与
C2/C3 的 CARLA 验收并行，但必须在统一 Scene Package、ROS 2 topic 和同步
时钟契约下汇合。

## 6. 最终完成定义（Definition of Done）

项目只有在以下条件全部满足后，才能标记为“完整闭环已收敛”：

- Scenario IR、Scene Package、运行配置、控制契约和报告 Schema 已冻结。
- 本地离线构建、esmini 验证、dry-run 和全部回归测试通过。
- 真实 CARLA BasicAgent 固定场景至少连续成功三次。
- 至少一种 scripted 或 TrafficManager Actor 在真实运行中产生可证明的交互
  反应。
- 真实 ROS 2 链路能够接收观测、输出控制，并在超时或断连时 safe-stop。
- 至少接入 TCP 或 TransFuser 中的一个真实 checkpoint，与 BasicAgent 使用
  同一场景、seed 和 KPI 契约完成横向比较。
- 同一算法可在不同 ODD 中比较，实验矩阵没有缺失、重复、未知 KPI 或伪交互
  运行。
- 核心 KPI 来源可信，缺失值不会被误判为通过。
- 所有运行均可追溯到场景版本、代码 commit、环境版本、算法镜像和权重摘要。
- 失败路径可诊断，CARLA settings、Actor 和传感器能够可靠清理。
- README、运行手册、已知限制、环境依赖清单和发布证据完整。

截至 2026-07-14，项目满足“离线实现完成、等待环境接入”，尚未满足上述
最终 DoD，因此不能宣称真实 CARLA/ROS 2 完整闭环已经验证。

## 7. 明确停止项

收敛期间不开展：

- 同时接入多个新的学习算法，或在本仓库重新实现算法模型。
- NuRec 任意新视角、任意 Ego 偏航后的神经渲染保证。
- 通用场景编辑平台、复杂 Actor 规则引擎或 ScenarioRunner 替代品。
- 新场景家族、大规模参数搜索、Web UI、云调度和分布式仿真。
- 与 TriggerEngine 或 NeuralSceneBridge 已有职责重复的数据处理。

新的功能设想只有在阻塞最终 DoD 时进入当前计划；其余统一进入发布后的
backlog。

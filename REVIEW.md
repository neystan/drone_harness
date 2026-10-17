# drone_harness 项目交接概览

最后更新：2026-10-17（Asia/Shanghai）。当前待办见 [STATUS.md](STATUS.md)，固定协作规则见 [AGENTS.md](/home/stan/.codex/AGENTS.md)。

## 项目与架构

项目使用同一个多模态 VLM 理解导航指令、观察 RGB-D 并调用飞行工具，在 ROS 2、MAVROS、PX4 与 AirSim 中形成闭环。

执行链：用户指令 → runtime / agent_loop → 主 VLM → tool_dispatcher → 飞行工具 / Px4Controller → 工具反馈 → 主 VLM。

| 位置 | 职责 |
| --- | --- |
| `/home/stan/AirVLN/drone_harness` | 当前开发仓库，GitHub `neystan/drone_harness`，分支 `main` |
| `/home/stan/AirVLN/drone_agent` | 原实现，只读参考 |
| `/home/stan/AirVLN_ws/AirVLN-theta-star` | 场景启动、数据集和评测适配 |
| `/home/stan/AirVLN/drone_harness_runtime` | 批测结果、日志、轨迹、视频和诊断证据 |
| `/home/stan/AirVLN/hw_ros2_native` | AirSim ROS 相机桥工作区 |

sim/real 入口为 `drone_harness_sim`、`drone_harness_real`；批测入口为 `scripts/run_sim_batch.py`。

## 已实现能力与运行边界

- 阶段 1 飞行闭环、阶段 2 导航清单已完成。当前业务基线为 `f7b0ea1`（精简分段提示）；测距 `c6f8667`、前下视角 `df0ac76` 已推送，后续业务及文档提交尚未推送。
- 正式入口有十个工具：`measure_distance`、`observe`、`get_state`、`takeoff`、`forward`、`up`、`down`、`rotate`、`land`、`update_navigation_plan`。
- `observe(prompt, view="front")` 支持 `front/down/both`，逐图标注摄像头视角；前视附配对深度，下视只有 RGB。新任务和动作后不自动附图，动作后观察主要靠提示词引导。上下文保留最新一次所选图组，新观察替换整组；子目标完成后清理旧工具协议与旧观察派生文字，保留自然语言、最新图和当前计划。
- 导航清单由主 VLM 创建和更新，程序只校验结构；任务可在同一进程接受用户引导续接。当前提示词为基础 system 加四段固定导航规则：导航清单、参照物观察、位置核验、接近核验。先执行指令要求的动作，再按任务选择观察或测距。
- 仿真 `forward` 每次重新获取 RGB-D，决策视距 20 米、预留约 1 米净空，单次最多 19 米；缺少有效深度或上限为零时不发布前进目标。`up/down` 单次最多 10 米，没有累计高度封顶或上下避障。成功运动后默认等待 2 秒，悬停后复用原 Offboard 恢复流程。
- `measure_distance(x, y)` 按需读取最新前视观察的配对深度，不新取图或移动，动作后须重新观察。归一化选点的 5×5 邻域至少 80% 有效才取中位数；P90−P10 及中心偏差按 max(0.3 米, 中位数 5%) 检查。可信超出共用 20 米上限时返回 `OUT_OF_RANGE`、`valid=false`、`distance_m=null`；缺测、混杂或饱和值保留未知。
- 测距是采图时相机到所选可见表面的射线距离，用于需要距离证据的接近核验，不作为常规前进前置步骤。局部深度有效不证明选对目标、可以直接前进或已经到达；`observe` 障碍摘要也不是目标距离。
- `get_state` 提供连接、解锁、模式、空中状态、NED 位置及参考高度，当前不返回航向；参考高度不是实时地面测距。实机动作保留人工审批。
- 模型请求超时 600 秒且不自动重试，普通轮次最多 50 次请求、导航轮次 200 次；批测每 case 最多 1800 秒。批测默认三维起点容差 5 米、航向容差 5°，显式配置可覆盖；独立场景启动器默认位置容差仍为 0.5 米。

## 当前实施方向

四向搜索 skill、全局反思机制和工具返回精简均已确认方案、尚未实施；完整范围及顺序只维护在 [STATUS.md](STATUS.md)。

搜索使用 VLM 动态生成的本次搜索重点，程序完成四向扫描并恢复开始朝向，再一次提供四图；普通观察和四图共用全局目标核对规则及 VLM 的 `target_status/message`。工具执行结果与语义判断分开，现有工具 `message` 原样保留。

始终先核对正确参照物，再结合实际动作、前下视角和按需测距核验位置与完成条件。参照物确认、模型勾选清单和独立评分成功分别记录；单张图片看到目标不证明已经经过、越过、穿过或位于其上方。当前业务代码尚无 skill 或 `target_status` 支持。

## 重要验证与证据

### 离线检查

2026-10-17 最近完整 harness 回归为 414 项通过。基础 system 为 429 字符，四段导航提示合计 1027 字符（此前 1481 字符），字符数减少不代表 token 或延迟实测。起点容差调整时 25 项离线测试通过；历史 10 条预检按新阈值复核满足条件，Case 55、173 尚未实际重跑。

### 最近正式批测

批次 [20261011-231257-222381](../drone_harness_runtime/batches/20261011-231257-222381/) 使用旧代码 `64befa7`。10 次尝试包含 2 次预检失败、1 次起飞失败、5 次飞行超时、1 次评分成功、1 次模型完成但评分失败。SR=0.1、OSR=0.1；平均 NE=58.4322 米，缺失 2 条，均值包含未起飞的 case 180。数据见 [results.csv](../drone_harness_runtime/batches/20261011-231257-222381/results.csv) 和 [summary.md](../drone_harness_runtime/batches/20261011-231257-222381/summary.md)。

- Case 55、173：RGB 和航向正常，初始垂直偏差约 2.44/2.57 米导致预检失败，证据为各运行目录的 `scene/preflight.json`。
- Case 17：模型先声称已右转并看见红卡车，随后才实际观察和旋转；见 `00009-r1-case17/agent_logs/session_20261012_014955/` 的消息和工具日志。
- Case 66：以屋顶画面位置推断已在建筑上方，并将障碍距离当作建筑距离；最终 NE=40.18 米。见 `00010-r1-case66/agent_logs/session_20261012_022119/`。
- Case 328 评分成功，NE=15.48 米，仅为单次成功证据。录像分析为关键帧抽查，未完整逐帧审阅全部视频。

当前成功条件是正常结束且最终水平 NE ≤ 20 米；功能短测、模型宣布完成和实机安全验收不能代替该评分。

### 测距与视角实测

- 2026-10-17 scene 24 / case 351 首轮：97 次 observe 尝试成功 96 次、14 次真实模型请求；完成 3 个自然目标选点及 25 米超范围反馈。3/8/15/19.9 米标靶中心最大误差约 0.0094 米，20/25 米均拒绝准确读数；实际前移约 0.94 米后墙面读数减少约 0.96 米。见 [首轮汇总](../drone_harness_runtime/diagnostics/20261017-204801-measure-distance-live/summary.json)。
- 首轮本地测距处理耗时中位数 0.178 毫秒；测距后的模型请求另需 18.89–23.93 秒。细目标补测为 20 次独立 VLM 选点、19 次测距、11/11 次 observe 成功：排水管仅 1/5 明确点在管身，凳腿有效读数落在背景，指定电线未明确点中。见 [补测汇总](../drone_harness_runtime/diagnostics/20261017-211127-landmark-point-stability/summary.json)。
- 楼体 5/5 点在建筑或附属表面，但部位变化；同图前三次读数为 8.44/6.65/5.44 米，不能当作同一点波动或接近证明。输入为生产 640×480、JPEG80、detail=low；补测临时上仰 60°、未起飞，结束已恢复相机并关闭仿真。该结果不代表正常前视下的远距离楼体导航。
- 前下双图配对已有实测；下视 `CameraDown` 为 640×480 RGB、Pitch=-90°，topic 为 `/airsim_node/PX4/CameraDown/Scene`。适配由 `scripts/aerialvln_batch_adapter.patch` 交付，历史干净基线应用补丁及适配器评测 21 项通过。飞行高度下的下视方位和语义尚未验证。
- 未决边界：首轮一次取图超时根因未定位；ROS 内参对应 FOV 94°，相机 API 约 89.904°，差异尚未核实。细目标错点和楼体选点变化不能靠深度一致性校验解决。实测已结束，正式批测成绩未改变，未进行实机飞行。

### 数据与方法调研

已核对 531 条指令，排除 scene 7 的 42 条后分析 489 条（178 条参考轨迹）；分类可重叠，统计不是动作次数。依据位于 `../drone_harness_runtime/analysis/20261016-case-taxonomy/`。终点与参考轨迹仅用于离线分析和独立评测，不提供给导航 VLM。

SPINE、UniGoal、DV-VLN、CLOSER-VLN 和 CoVe 的机制、来源及调用开销保存在桌面批测说明。这些调研支持设计取舍，不能证明当前单 VLM 反思方案已经提升导航效果；未引入上游模型、地图或新依赖。

## 代码与资料入口

| 位置 | 职责 |
| --- | --- |
| `drone_harness/runtime/agent_loop.py` | 模型与工具循环、观察注入、上下文和请求预算 |
| `drone_harness/runtime/navigation.py`、`task_memory.py` | 导航清单、任务续接与子目标记忆 |
| `drone_harness/runtime/tool_dispatcher.py` | 工具分发与执行前处理 |
| `drone_harness/runtime/observation.py` | RGB-D 配对和图文消息构造 |
| `drone_harness/tools/flight.py`、`px4/controller.py` | 飞行动作和 PX4 控制 |
| `drone_harness/llm/prompts.py`、`client.py` | 提示词和唯一模型客户端 |
| `drone_harness/testing/batch.py`、`scripts/run_sim_batch.py` | 批测流程和入口 |
| `drone_harness/config/profiles/sim.yaml`、`real.yaml` | 仿真与实机配置 |

桌面 [项目全景](/home/stan/桌面/AirVLN-drone-agent-项目全景.html) 展示架构与进展，[仿真批量测试](/home/stan/桌面/仿真批量测试.md) 保存批测操作、详细结果和调研。README 与阶段实施文档仅供背景参考，实际行为以代码和当前待办为准。

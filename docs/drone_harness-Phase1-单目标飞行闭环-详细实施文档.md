# `drone_harness` Phase 1：单目标飞行闭环详细实施文档

> 2026-09-25；用户已批准从 S1 开始实施，并追加授权在各阶段自检通过后**连续实施至 S7**，无需逐阶段等待回复。每阶段仍须独立测试、验收和提交，不能跳步；遇到安全、资产或服务阻碍则按该阶段停止条件停下。依据同目录《drone_harness-Phase1-单目标飞行闭环-设计规范.md》，按用户提出的七步细化。真机飞行仍未获授权。

## 0. 目标、解释与边界

目标是把**人工给定的一个目标**交给一个持续运行的 `drone_harness`：每轮取得时间匹配的新 RGB-D，程序产出简短深度规则和数值安全上限，同一个多模态 VLM 看 RGB、读规则并用原生 function calling 提议**至多一个**短飞行动作；程序检查、执行，再自动补入动作后的新观测。双入口改名为 `drone_harness_sim`、`drone_harness_real`，保留各自 profile 及 PX4/MAVROS 安全链。真机入口只做配置/HITL 桩件验证，不进行真机自主飞行。

用户第 6 步的“单一 VLM 模型进行规划、工具调用与多模型”按已批准设计规范理解为“**单一多模态 VLM**”：图片理解、文本规划、function calling 在同一模型/服务完成，不接第二个视觉模型。如果“多模型”意指多模型协同，须先修订设计规范，再实施；本计划不擅自增加模型。

“每次 `agent_loop` 前注入”有两个时点：新目标进入 `agent_loop` **之前**注入首个观测；同一个 `agent_loop` 内每次成功的非终止动作之后、**下一次 VLM 请求之前**再注入动作后的新观测。只在下一次交互式 runtime 调用前注入，会让一次 function-calling 循环里的后续决策继续看旧图，不满足闭环要求。这里的“同一帧”指 RGB 与深度来自**经时间阈值配对的同一次观测**，而不是两个 ROS topic 必须有完全相同的时间戳。

原 `/home/stan/AirVLN/drone_agent` 始终是只读来源：不得编辑、清理 dirty、重置、提交或更改 remote。新 `drone_harness` 仓库才是改造目标。阶段 1 不包含长指令拆解、任务队列、语义地图、复杂局部规划或真实飞行许可。

**最小改造原则。**保留现有 CLI 两入口、`runtime → agent_loop → tool_dispatcher → flight → Px4Controller` 调用链、`TaskState`、消息总线、ROS executor 和 PX4 安全处理；只在这些位置加闭环必需的观测、单模型调用与前进检查。除小型 RGB-D 缓冲/深度规则纯计算模块外，不新增 Agent、ROS 节点、任务调度器、安全管理器、监视线程或模型可见观察工具。

**代码与提交纪律。**所有新增函数和方法（包括测试辅助函数）都写一句简洁的中文注释或 docstring，说明职责或关键约束。S2–S7 每阶段只形成一次提交，标题采用示例的 Conventional Commit 形式 `type(scope): 中文简述`，正文用中文说明动机、变更边界、测试命令与结果、未解决的限制；不把密钥或原始图像写入提交。S1 已在本要求提出前公开提交，不强推改写历史。各阶段的失败不能用提交信息写成通过。

**命名与授权。**S2 在新仓库内统一 `drone_harness` 包、ROS 包、命令入口和当前运行文档；原来源路径与 S1 迁移事实保持原名。用户已同意必要的真实 VLM 服务探针和仿真测试，指定 `glm-5.3-flash`；服务密钥只在临时进程环境使用，绝不写入版本控制、任务日志或测试快照。仿真仍须满足 S7 的只读预检、独立运行目录与保守标定门槛；此授权不包含真机飞行，也不允许清理别的仓库 dirty 文件。

## 1. 现状锚点与七步顺序

当前实现的对应点：`runtime/agent_loop.py` 会在同一模型回复中逐个分发全部 `tool_calls`；`runtime/runtime.py` 在交互输入前只追加文本，主客户端是 `llm/client.py`，`analyze_view` 使用另一条 `vision/vlm.py` 链；`px4/controller.py` 只缓存 `latest_rgb_frame`、没有同步深度；`tools/registry.py` 暴露 17 个旧工具；`tools/flight.py` 的 `move(x,y,z)` 是三轴底层动作；`config/profiles/real.yaml` 对 `rotate`、`land` 仍有 HITL 豁免。已有 `runtime/safety.py` 的确认悬停和 PX4 失控交接必须保留。上述路径均指**迁移到新仓库后的副本**，不是授权修改来源目录。

| 步骤 | 用户给定主线 | 本步可单独验收的结果 | 尚不可做的事 |
|---|---|---|---|
| S1 | 建立公开仓库 | 独立、public、受控迁移和基线记录 | 不开始代码重构 |
| S2 | 删除多余旧内容 | 旧模型工具链不可达，内部安全链仍在 | 不开放中间态自主飞行 |
| S3 | RGB-D 结构化信息 | 同步观测、几何摘要、数值上限可离线复现 | 不让模型凭暂定余量飞行 |
| S4 | runtime 观测注入与提示词 | 假 VLM/假工具完成逐轮消息顺序 | 不接 PX4 执行 |
| S5 | 四动作安全门 | 四工具、前进动态上限、运动中失效悬停、真机全 HITL | 不以桩件证明真机可飞 |
| S6 | 单一多模态 VLM | 一个服务同时接 RGB、规则与四工具并继续对话 | 未验证服务前不跑仿真 |
| S7 | 测试 | 故障回放、完整离线回归、单目标仿真证据 | 不做真机飞行 |

**顺序原则。**七步顺序遵循用户指定，取代设计规范第 8 节的原“建议顺序”，但不改变该规范第 1–7 节的架构和验收边界。S2 先精简与统一命名；S2–S6 只做离线测试和无飞行服务探针，未完工时不启动实际运行入口，占位 `forward` 必须拒绝执行，无需新增长期存在的阶段开关。测试也不是等到 S7 才开始：每步必须运行其聚焦测试，自检通过后再继续并提交一次；S7 做跨模块、故障回放和仿真总验收。每步单独记录改动清单、测试命令/输出、失败原因和来源只读核对；不得用后面的演示掩盖前一步失败。

## 2. 跨阶段接口合同

这些是实现时须保持稳定的**数据关系**，不要求为每一项新增类。优先复用现有 `ToolContext`、`TaskState`、工具返回 `dict` 和日志函数；仅为 RGB-D 观测及规则增加确有需要的数据容器。

| 合同 | 最少字段/行为 | 失效时 |
|---|---|---|
| `ObservationSnapshot` | `observation_id`、RGB 内容、深度数组、各自采集时间、接收时间、相机标定、位姿/状态快照、同步/新鲜度标记 | RGB 不可用则停止本轮；深度不可用可形成显式失效摘要 |
| `DepthRules` | 与快照相同的 `observation_id`、`depth_valid`、`front_clearance_m` 或未知、`forward_max_m`、左前/右前可见障碍或未知、原因 | 不可信时 `forward_max_m=0`；不能沿用旧深度 |
| 现有 function calling | 原生的一个 `takeoff/forward/rotate/land` 调用，或零调用文本；沿用现有 tool-result `dict` | 多调用整体拒绝；未知/坏参数零执行；失败后不继续飞行 |

安全检查直接在现有 `runtime/tool_dispatcher.py` 和 `tools/flight.py` 的调用边界完成，读取 `ToolContext` 中的最新观测规则、`profile.safety` 和 controller 状态；结果写入现有工具日志。**不另造 `SafetyDecision`、`ActionProposal`、`ActionResult` 三套对象或第二个调度器。**

成功的非终止动作之后，消息顺序固定为 `assistant.tool_calls(1) → tool 结果 → 新的多模态 user 观测 → 下一次 VLM 请求`。`land` 成功即结束；模型零调用的“候选完成”只是候选，必须独立核验且不会自动触发 `land`。长任务只保留当前目标、最近动作结果、步骤/观测编号、连续拒绝/无位移计数及有界消息历史，不引入第二阶段的任务队列。

## S1. 建立 public 仓库并受控迁移基线

**输入与只读预检。**实施开始时记录来源 HEAD、remote、`git status --porcelain`、待迁移文件清单及哈希；核对 `/home/stan/AirVLN/drone_harness` 是否已有内容、GitHub 当前账号和同名仓库是否存在。当前来源有本地 dirty 与未跟踪测试，迁移基准是**实际工作文件**而非单纯 HEAD。账号不明、目录/仓库冲突或来源状态与审阅时不符，先停下询问，不覆盖、清理或擅自换名字。

**实施拆分。**① 新建空 public GitHub 仓库和独立本地目录；先确认可见性。② 只迁移 `drone_agent/` Python 包、`test/`、`scripts/drone_agent_sim`、`scripts/drone_agent_real`、必要的 `launch/`、`resource/`、`package.xml`、`pyproject.toml`、`setup.py`、profile 和当前任务所需文档；来源 `.git` 不迁移。③ 审查每个来源 dirty/未跟踪文件是否属于运行基线并记来源哈希，不能把用户未授权的其他工作顺手推送。④ 添加只覆盖本项目的忽略规则，排除真实 `settings.json`、API key、运行图像/视频、日志、数据集、仿真资产、ROS/PX4 安装、虚拟环境、build/install/log 输出。⑤ 在任何首次 push 前检查暂存清单、可能的凭证和文件体积；不通过就不推送。原 private 默认值已由用户于 2026-09-25 明确改为 public。

**拟变更位置。**只在新仓库的根目录、打包文件、profile 示例和迁移记录中操作；暂不改飞行、观测或 VLM 逻辑。S1 时尚未要求重命名 Python 包或入口；该要求在 S2 落地。

**测试与证据。**在新目录记录 `python -m pytest -q` 基线、包导入、profile 解析、两个入口的安装元数据；只读核对来源 HEAD/remote/status/哈希前后一致。提供 GitHub public 可见性证据和待提交文件清单，屏蔽账号凭证。若某测试因 ROS/设备缺失无法运行，明确标为“环境阻碍”，不得记为通过。

**通过/停止。**public 远端及本地新目录独立存在、迁移清单可复核、两入口保留、没有敏感/大资产入库、来源无变化才通过 S1。任何目录冲突、隐私扫描失败或来源状态变化都停止；不能进入 S2。

## S2. 删除旧内容并统一 `drone_harness` 命名，但保留安全底座

**输入。**S1 通过，并先对旧模块做导入/调用依赖审计。这里“删除 `drone_agent` 多余内容”仅指**新仓库里的迁移副本**，绝不删除来源仓库文件。

**删除/保留判据。**模型工具注册表中移除 `activate_skill`、`disarm`、`timer`、`hover`、`return_home`、`current_position_status`、`battery_status`、`flight_mode_status`、`move`、`take_photo`、`analyze_view`、`detect_target`、`sam_tracking`、`mouse_tracking`。其中悬停、RTL、状态读取、人工接管可以继续是**程序内部能力**，只是 VLM 不可自由调用。对 `tools/perception.py`、`tools/tracking.py`、`tools/skill.py`、旧 `vision/{vlm,dinoxseek,tracking,...}.py`、`skills/` 和相关依赖/config 做引用搜索；只删除确实无消费者的运行链，不顺带重写 `px4/controller.py` 或移除测试需要的安全接口。历史文档若保留，须标明不再是当前运行架构。

**中间态处理。**保留 `takeoff`、`rotate`、`land` 的底层实现；`forward` schema 可先占位，但处理函数必须返回明确的 `FEATURE_NOT_READY`，不能调用 `move`。S2–S6 只用假 controller/假工具做离线测试，不启动实际运行入口；无需为这个施工中状态新增长期存在的“执行门”子系统。旧文本 LLM 客户端在 S6 迁移前可作为代码依赖暂存，但不可据此恢复旧多工具自主飞行链。

**统一命名。**仅在新仓库将 Python 包、ROS package/resource、打包元数据、两个脚本与 console entry points、内部 import/模块启动命令及当前 README 改为 `drone_harness` / `drone_harness_sim` / `drone_harness_real`。迁移记录中真实的旧来源路径与提交信息不改写；过时架构文档移至历史区或移除，不把旧工具用别名重新暴露。新增入口和导入测试证明两种 profile 仍可加载。

**拟变更位置。**`tools/registry.py`、`tools/schemas.py`、相关 `tools/` 与 `vision/skills/` 文件、`runtime/runtime.py` 的旧 skills 装载、`setup.py`/`pyproject.toml` 的确无用依赖，以及 Python/ROS 包目录、入口脚本与当前 README；不碰来源。旧视觉图片编码若后续单 VLM 仍可复用，应抽成纯工具函数后再移除旧 `analyze_view` 调用链。

**测试与证据。**增加工具注册表测试：可见集合最终为四个名字（其中 `forward` 此时 fail-closed），旧工具名全部不可调度；用假控制器验证占位 `forward` **零 PX4 命令**。运行已有安全 ACK、MAVROS、HITL 测试；与被删功能直接绑定的旧测试需逐项标注“删除原因/替代断言”，不能简单删除全部失败用例。用引用搜索与打包文件清单证明无悬空 import，保留内部 `request_confirmed_hover`/RTL/failsafe。

**通过/停止。**运行时不再调用独立 `analyze_view`、检测、追踪或 skill，旧工具不可达且安全底座回归通过，S2 才通过。若清理破坏启动、状态监控或应急处理，先恢复/修正新仓库代码，不进入 S3。

## S3. 输出 RGB-D 结构化规则和前进数值上限

**输入。**S2 通过；先查证仿真 RGB/depth topic 的消息类型、编码、时间戳域、深度语义（例如平面深度或透视距离）、单位及相机内外参。仅有一个叫 `CameraDepth1` 的 RGB topic **不证明**已有深度消息。真实 profile 不能借用仿真 topic/标定当作已验证输入；未核实的真机深度一律无效、前进上限为 0。

**采集与配对。**在现有 `px4/controller.py` 节点上增加最小的深度/相机参数订阅，用小型 `runtime/observation.py` 辅助缓冲/配对，**不另建 ROS 节点**；继续使用现有后台 ROS executor，让原 setpoint timer 在等图和 VLM 时运行。回调只做解码、时间标记和有界缓冲，不做耗时模型请求。按同一 ROS 时钟域的 header 时间配对 RGB 与深度；检查最大配对时差、最大帧龄、缓冲大小和等待超时；同时记录单调时钟接收时间以测等待耗时。动作后新观测必须能证明其**采集**晚于动作结束，不能仅因旧帧“晚收到”就视为新帧；时钟域不可比较则停止该轮。缺深度可返回带错误码的 RGB 观测，但不伪造可前进距离。

**几何与摘要。**拟增加纯计算 `vision/depth_rules.py`：校验 `16UC1/32FC1` 等实际编码和单位、过滤 NaN/Inf/零/超量程，用已确认的相机模型转换到机体坐标；按机体宽高和测量余量检查短前进通道及覆盖率，不能只取中央单像素或把未知区当空地。`front_clearance_m` 定义为**机体前缘到最近可信障碍的净空**；`forward_max_m = max(0, min(profile.safety.max_relative_move_m, front_clearance_m - 测量余量 - 制动/超调余量 - 延迟余量))`，机体前缘偏移已体现在净空定义里，不重复扣减。复用现有 `max_relative_move_m` 字段，但须把 sim 当前的 20 m 值收紧为待仿真标定的**短步限额**；不能原值直接放行。第一版只输出 `observation_id`、`depth_valid`、`front_clearance_m`、`forward_max_m`、左前/右前“可见障碍/未知”及原因；这些不是目标距离或全向通行证明。深度几何余量仍需少量显式配置，S7 标定之前不放行自主前进。

**拟变更位置。**`config/schema.py`、`config/loader.py`、`config/profiles/{sim,real}.yaml` 加观测 topic、同步/时效/相机语义与限额配置；`px4/controller.py` 接消息；新建 `runtime/observation.py`、`vision/depth_rules.py`（文件名拟定）；`logging/task_log.py` 只记摘要与元数据，不默认保存完整帧。

**测试与证据。**拟增 `test/test_observation_buffer.py`、`test/test_depth_rules.py`、相关 ROS stub 测试：近时/错时、旧帧晚到、重复帧、掉帧、时钟域不明、等待超时、深度单位/编码错误、细障/边缘障/通道外障、未知覆盖、NaN/Inf/0、净空边界和“障碍更近则上限不增”。纯合成输入即可运行，不开仿真。检查 `depth_valid=false` 的文本与数值对象都给 `forward_max_m=0`，且摘要与 RGB 的 `observation_id` 一致。

**通过/停止。**可用合成 RGB-D 稳定得到可追溯的观测和规则；任何缺失、错配、过期、未知单位/标定都不能给正前进上限。深度源或相机几何不明时，只能通过 fail-closed 输出的测试，不能宣称测距已可用，也不能进入 S7 自主前进。

## S4. 修改 runtime：自动注入观测、轻量状态与提示词

**输入。**S3 的观测/规则合同通过；本步只用假 VLM 和假动作结果验证消息循环，不启动实际运行入口。保留一个持续运行的 runtime，不改成“每动作重新启动 runtime”，也不把拍照变成模型工具。

**首轮注入。**新目标到来时，`runtime/runtime.py` 先建立该目标自己的消息上下文：固定 system prompt、当前目标和首个 `ObservationSnapshot`。RGB 用服务支持的图片内容格式直接放进多模态 `user` 消息；同一观测生成的深度摘要以文本放在**同一条**消息中，带 `observation_id` 和采集时间。没有有效 RGB 时不调用 VLM。深度失效时可发送 `depth_valid=false; forward_max_m=0; reason=...`，但程序安全门仍独立禁止 `forward`。

**动作后注入。**`runtime/agent_loop.py` 一次只接受 0 或 1 个工具调用。合法单动作的结果按协议追加为 `tool` 消息；如果它是成功的非终止动作，立刻等待**采集晚于动作结束**的新 RGB-D，并在下一次模型请求前追加新的多模态 `user` 消息。若动作失败、等待新 RGB 超时、用户中断或控制状态不可信，结束本次自主循环，不拿动作前的画面继续规划。若 `land` 成功，不再调用导航 VLM。若模型返回多个 `tool_calls`，**任何一个都不执行**；若决定保留该回复在历史中，就为每个 call ID 写入拒绝结果保持工具协议配对，否则丢弃这次回复并结束该轮。

**消息与状态预算。**一个人工目标对应一个有界对话；runtime 可继续接下一条人工输入，但不能把上个目标的图片无限继承。保留 system、目标、当前观测、上个动作及其成对的工具结果、必要的简短状态；旧图片与冗长原始返回移出 `messages`，日志可留去敏元数据。扩展 `runtime/task_state.py` 记录 `observation_id`、step_id、最近动作实测结果、连续拒绝/无有效位移次数及最大轮次。所谓“无进展”第一版只依据可核对的动作失败或位姿无变化计数，不把 VLM 自报“接近目标”当安全事实；超过预算就安全停止。

**提示词内容。**拟更新 `llm/prompts.py`，明确：只处理当前单目标；RGB 用于识别，距离/净空以程序深度规则为准；仅 `takeoff/forward/rotate/land`，每次最多选一个；每次动作后等待工具结果和新观测再决定；深度无效/未知、看不清或图文矛盾时不得前进；不得虚构视野外/背后/着陆区安全；不调用检测、追踪或 skill；疑似到达时**不调用飞行工具**，只用文字报告“候选完成”和可见证据。提示词是行为引导，不代替程序限额或授权。

**拟变更位置。**`runtime/runtime.py`、`runtime/agent_loop.py`、`runtime/task_state.py`、`llm/prompts.py`、`logging/task_log.py`；可以增加纯消息构造器，供离线测试。S4 不接真实客户端、不开 PX4。

**测试与证据。**拟增 `test/test_agent_loop_observation.py`、`test/test_message_compaction.py`：假客户端记录请求，断言 `首观测 → VLM → 单工具 → tool 结果 → 新观测 → VLM` 顺序；新观测确实晚于动作完成且 RGB/摘要同号；拒绝多调用之前执行次数为 0；无调用、候选完成、未知工具、坏参数、动作失败、RGB 超时、失效深度、用户中断、`land` 成功均有正确停止语义；长轮次下图片数和消息长度有界。现有 `test/test_mavros_sim_runtime.py` 的 setpoint/线程安全回归不能因改循环失效。

**通过/停止。**从假客户端记录能逐轮复原“一新观测—一次决策—最多一个动作—动作后新观测”；一个 runtime 内可连续多轮，且假控制器的命令计数仍为 0。任何路径出现旧帧续飞、多动作执行或旧图无限累积，则停止修正，不进入 S5。

## S5. 四动作安全门与执行中失效保护

**输入。**S4 的调用边界通过，S3 给出与观测号绑定的数值上限；先复核现有 `flight.move` 的轮询、PX4 setpoint timer、`runtime/safety.py` 的 `request_confirmed_hover` 与人工中断语义。本步仍先在假控制器上验证，不启动实际运行入口。

**四动作表。**

| 模型动作 | 参数与执行前检查 | 底层执行和终止 |
|---|---|---|
| `takeoff(height)` | 沿用现有有限正值、高度限额、起飞状态/电池/PX4 就绪检查 | 原实现不重写；成功后再取新观测 |
| `forward(distance_m)` | 新增正向距离、同次深度有效/新鲜且不超动态上限检查 | 包装现有 `flight.move(context, distance_m, 0, 0)`；成功后再取新观测 |
| `rotate(direction,degrees)` | 沿用现有方向、有限角度、profile 限额、位姿/飞行状态检查 | 原实现不重写；转向后须取新观测再前进 |
| `land()` | 在原位姿/飞行状态/PX4 确认前提上加明确降落授权；候选完成不等于授权 | 原实现不重写；成功为终止动作 |

**最小新增检查。**不创建 `SafetyDecision` 类或第二套调度器；`forward` 包装器直接读取 `ToolContext` 中的最新 `DepthRules`、现有 `profile.safety.max_relative_move_m` 和 controller 状态，复用现有 `is_finite_number`。超限、零/负、布尔值、NaN/Inf、深度未知或旧观测都拒绝并用原工具日志记录，**不静默截短**；已有 `move()` 仍负责位置、航向、机体系换算、飞行状态和超时检查。VLM 只见 `forward`，无法给底层 `move(x,y,z)` 设置 `y/z`、后退或侧移。明确降落授权可在现有 `TaskState` 中用一个状态位或本次人工确认承载，不新增授权服务，也不从 VLM 的“候选完成”推断。真机沿用 `runtime/tool_dispatcher.py` 的 HITL 流程，只把 `real.yaml` 的 `rotate/land` 豁免清空，并在确认后执行前重查观测/上限和状态；提示中补动作、参数、观测号及有效期。内部应急悬停、RTL/失控保护不等待模型或 HITL。

**复用 `move()` 轮询。**不增加监视线程或独立安全控制器；仅在现有 `move()` 的每次轮询中插入一个可选的 `forward` 检查，读取最新深度/位姿。新障碍、深度或位姿失效/过期时，复用现有 `request_confirmed_hover`；PX4 未确认就沿用其 Offboard-loss 安全交接。现有超时分支已经调用该函数，保留不重写。现有 `interrupt_if_requested` 的人工中断分支只发送 hover、**不确认** PX4 状态；为满足本计划对 `forward` 中断的要求，只补该动作的中断分支复用 `request_confirmed_hover`，不扩建安全框架。检查在动作线程的原轮询中进行，不阻塞 ROS executor 的 setpoint timer。轮询间隔、最长盲行距离与现有 PX4 实际速度在 S7 标定；若无法证明守卫来得及制动，则不放行自主前进。

**拟变更位置。**主要是 `tools/schemas.py`、`tools/registry.py`、`tools/flight.py` 的 `forward` 包装与 `move()` 轮询，以及 `runtime/tool_dispatcher.py` 的批准后重检；`runtime/safety.py` 仅更新模型飞行动作名单，`config/profiles/real.yaml` 清空豁免。深度读取复用 S3 的观测缓存；原则上不改 PX4 控制器的 setpoint/Offboard 逻辑，也不增加新的安全模块。

**测试与证据。**拟增 `test/test_forward_gate.py`、`test/test_motion_guard.py`、`test/test_real_hitl.py`：参数边界、动态上限缩小、审批期间过期、转向后的旧深度、位姿丢失、运动中障碍/断流、PX4 拒绝、超时、用户介入、悬停 ACK 被拒和安全交接。断言正向底层调用恒为 `y=z=0`、任何拒绝都未发布危险目标点、失效后没有下一次模型动作。继续运行现有 `test/test_flight_safety_ack.py`、`test/test_mavros_controller.py`、`test/test_hitl_flight_tool_exemptions.py` 的等价新断言；原“允许豁免”测试应改为“真机四动作无豁免”，不是简单删掉。

**通过/停止。**四动作可见面恰好匹配设计，真机四动作 HITL 全覆盖，前进限额与运动中守卫在假控制器上均 fail-closed，安全悬停/交接可被断言。任何超限仍可执行、审批绕过重检或失效只记录不处理，均不得进入 S6。

## S6. 用一个多模态 VLM 完成识图、规划和 function calling

**输入。**S5 通过；需要确定**一个**服务实际支持同一请求中的图片、文本和原生工具调用，以及包含工具结果后继续对话。服务/模型标识、图片尺寸限制、延迟、超时、费用和数据发送范围须在实施前核对；不得猜测现有文本 `llm` 或旧 `vision.vlm` 自动具备该组合能力。

**接线。**优先沿用现有 `RuntimeProfile.llm` 的 `base_url/model/api_key`、`llm/client.py::create_llm_client` 和 `client.chat.completions.create`，只把其服务换成可同时接图片与 function calling 的 VLM；删除独立可选 `vlm` 的运行依赖，不新建第二套 provider 抽象。S4 的多模态消息构造器提供最新 RGB 图片和深度规则；同一请求同时发送四工具 schema，返回 0/1 个 tool call。旧 `vision/vlm.py::analyze_image` 不再被调用，独立 DINO/SAM2 链不回流。图像尽量内存中编码，限制尺寸/字节数并记录哈希或观测号，不把原始 base64 写入任务日志。超时/网络错/服务不支持图片+工具/响应结构异常均安全停止，不回退到无图文本规划或第二个模型。

**真实兼容性探针。**先用假客户端跑完整多轮对话；获准使用服务后，仅用非敏感测试图和**无飞行、无 PX4**的四工具 schema 做真实探针：首请求含图片+文本+工具；返回 tool call 后插入假 tool 结果和一张新图；检查第二次请求被接受、响应可解析、费用/延迟在可接受范围。若探针不获准、供应商不兼容或过慢，停在 S6 选择服务/调整适配器，不启用仿真闭环。

**拟变更位置。**`config/schema.py`、`config/loader.py`、示例设置、`llm/client.py`、`runtime/runtime.py`、`runtime/agent_loop.py`、`pyproject.toml`/`setup.py` 中确实不再使用的依赖；清除尚存的旧双模型配置引用。真实 key 仅走本地设置/环境变量，不写入仓库或日志。

**测试与证据。**拟增 `test/test_multimodal_client.py`、`test/test_provider_config.py`：同一客户端/同一模型收齐图片、摘要、目标、四工具；继续对话消息顺序合法；无图片时不请求模型；服务异常零执行；旧配置不能悄悄激活第二模型。真实探针记录脱敏请求结构、响应模式和耗时，不存密钥/整图；本阶段不启动仿真。

**通过/停止。**无飞行证据证明“一个多模态 VLM + 原生 function calling”可工作，旧独立 VLM/检测链调用数为零；异常不会导致无图续飞。未完成真实兼容性探针时，S6 标为未通过，不进入 S7 的自主仿真。

## S7. 分层测试、故障回放与一个仿真目标

**输入。**S1–S6 均单独验收，服务兼容性通过，仿真场景、单一目标、目标到达判据、允许的运行时间/资源和日志目录已确定。若借用 `/home/stan/AirVLN_ws/AirVLN-theta-star` 的旧 AerialVLN-S × PX4 环境，先严格按其交接文档第 0、11 节做**只读预检**：两个仓库、运行资产、端口/进程与既有成功报告状态；该旧成功报告只是环境参考，不是本项目验收。端口占用或资产不明即停止请用户决定，不杀进程、不清 dirty、不覆盖既有运行目录。

**第 1 层：全量离线回归。**在新仓库执行 `python -m pytest -q`，保留 S1 基线对照；运行安装/导入、两个入口与 sim/real profile 的静态加载测试。逐项登记通过、失败、跳过和环境原因；不得通过删掉旧安全测试或降低断言让结果“变绿”。重新检查 public 可见性、Git 文件清单、敏感信息和来源仓库 HEAD/remote/dirty/文件哈希不变。

**第 2 层：固定故障回放。**至少覆盖：深度缺失/错配/旧帧/单位错误、近障/未知覆盖、RGB 超时、VLM 超时/坏响应/多调用、动态上限缩小、HITL 拒绝/超时/审批后观测变旧、PX4 拒绝/动作中断/悬停未确认、人工打断、`land` 未授权、候选完成。每个用例断言**后续模型调用数、飞控命令数、停止原因、观测号和安全交接**；不能只检查错误文本。对故障回放必须能证明没有危险后续动作。

**第 3 层：仿真标定与单目标闭环。**在新且唯一的运行目录，先用受控试验测深度误差、控制超调/制动距离、传感器与守卫延迟，写回 profile 余量和速度限额；若无法证明保守，停在标定，不让 VLM 自主前进。随后在**一个固定目标**上运行 `起飞 → 新观测 → 旋转/短前进 → 新观测 → … → 候选完成或安全停止`。逐轮保留观测号/时间、RGB-深度配对、深度文本与程序数值上限、VLM 请求/工具调用数、安全门结果、动作结果和 PX4 最终状态。图像/运行资产留本地受控路径，不推 GitHub。候选完成时先停止自主动作，用仿真真值或人工复核核验目标；不因模型自述自动判定到达、切换任务或降落。

**验收分级。**“安全闭环通过”要求离线故障回放、原安全回归和仿真逐轮时序均通过；仿真若安全停止，只能记为安全停止及未完成目标。“目标完成”还要求预先定义的单目标成功判据有独立证据。出现保护交接失败、旧帧续飞、多调用执行、无深度前进、超限前进或未经授权降落，S7 失败并保留原始证据。阶段 1 **不进行真机飞行测试**，`drone_harness_real` 仅验配置和四动作 HITL。

## 3. 验收矩阵与交付记录

| 设计规范第 7 节标准 | 主要证据所在步骤 |
|---|---|
| 独立公开仓库、来源只读 | S1 建仓/迁移记录；S7 再核对 |
| 一个图片 VLM、自动注入新 RGB-D、没有旧感知链 | S2 清理清单；S3 同步测试；S4 消息回放；S6 真实无飞行探针 |
| 恰好四动作、每回复最多一动作、`forward` 限额和零侧/垂直位移 | S4 多调用测试；S5 工具/控制器断言；S7 回放 |
| 失效处理、原安全链与真机 HITL | S3 失效规则；S5 ACK/HITL/动作中守卫；S7 故障矩阵 |
| 一个仿真目标的可复核闭环与独立到达核验 | S7 唯一运行目录、逐轮日志、真值或人工复核 |

每步完成后交付：① 新仓库的阶段提交；② 精确测试命令和原始结果；③ 通过/未通过与问题清单；④ 文件/日志证据路径；⑤ 来源仓库只读核对。用户已授权自检通过后连续实施，无需逐阶段等待；出现该阶段的停止条件、缺少不可安全推断的资产或需要真机飞行时仍须停下。付费 VLM 与仿真授权仅限本阶段约束，不能代替传感器/运动标定。**本实施文档本身不执行任何一步**。

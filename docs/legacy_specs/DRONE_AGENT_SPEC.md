# drone_agent 当前实现规格说明

日期：2026-06-15

## 1. 文档定位

这份文档描述的是 `/download/drone_agent` **当前已经落地的真实实现**，不是最初的目标态草案。

后续如果继续做架构演进、能力扩展或重构规划，应以这份文档作为基线，再在其上新增新的设计文档，而不是继续参考已经失真的旧版本描述。

## 2. 项目目标

`drone_agent` 是一个基于自然语言的无人机控制 Agent。

当前主线能力是：

- 用户在终端输入自然语言。
- LLM 通过 OpenAI 兼容 Function Calling 选择工具。
- 工具通过 ROS2 `rclpy` + `px4_msgs` + PX4 uXRCE-DDS 控制 PX4 无人机。
- 同一套核心控制逻辑同时支持仿真和真机。
- 相机图像可用于拍照和 VLM 画面分析。

## 3. 当前技术路线

当前主控链路是：

```text
输出终端
  -> drone_agent_sim / drone_agent_real
  -> runtime.runtime.start_runtime()
  -> 创建 TaskState 实例
  -> 创建 MessageBus 与 InputServer
  -> 自动拉起独立输入终端 input_terminal.py
  -> runtime.agent_loop.agent_loop()
  -> runtime.tool_dispatcher.dispatch_tool_call()
  -> TaskState 状态转移（thinking / tool_running / tool_completed / interrupted / intervention_pending）
  -> tools/*
  -> px4.controller.Px4Controller
  -> ROS2 rclpy + px4_msgs
  -> PX4 uXRCE-DDS
```

当前项目同时具备两种工程身份：

- GitHub 主仓
- ROS2 `ament_python` 包源码目录

## 4. 运行模式

当前对外保留的主入口只有两个：

```bash
drone_agent_sim
drone_agent_real
```

ROS2 工作区下也可以这样启动：

```bash
ros2 run drone_agent drone_agent_sim
ros2 run drone_agent drone_agent_real
```

其中：

- `drone_agent_sim` 固定加载 `sim` profile
- `drone_agent_real` 固定加载 `real` profile

虽然 `cli.py` 内部仍保留了 `--profile` 解析器，但当前公开使用方式就是这两个入口，不再以通用 `drone_agent --profile ...` 作为主使用方式。

## 5. 当前目录结构

```text
drone_agent/
  package.xml
  pyproject.toml
  setup.py
  setup.cfg
  README.md
  settings.example.json
  launch/
  resource/
  scripts/
  docs/
    legacy_specs/
    PROJECT_ARCHITECTURE.md
    PHASE_1_skeleton-config.md
    PHASE_2_px4-control-layer.md
    PHASE_3_tools-agent-loop.md
    PHASE_4_vision-safety-runtime.md
    PHASE_5_TASK_STATE_DESIGN.md

  drone_agent/
    __init__.py
    __main__.py
    cli.py

    config/
      __init__.py
      loader.py
      schema.py
      profiles/
        __init__.py
        sim.yaml
        real.yaml

    llm/
      __init__.py
      client.py
      prompts.py

    logging/
      __init__.py
      task_log.py

    px4/
      __init__.py
      controller.py
      frame.py
      status.py
      topics.py

    runtime/
      __init__.py
      runtime.py
      agent_loop.py
      safety.py
      tool_dispatcher.py
      task_state.py

    tools/
      __init__.py
      flight.py
      perception.py
      registry.py
      schemas.py
      status.py

    vision/
      __init__.py
      image_store.py
      prompts.py
      vlm.py
```

## 6. 各层职责

### 6.1 外层 ROS2 包壳

`package.xml`

定义 ROS2 包元数据和依赖。

`pyproject.toml`

定义纯 Python 打包方式、依赖声明和 console script 入口，与 `setup.py` 并存。`pyproject.toml` 是现代 Python 打包推荐方式，`setup.py` 是 ROS2 `ament_python` 兼容所需。

`setup.py`

定义 Python 包打包方式、ROS2 launch 文件安装，以及两个 console script：

- `drone_agent_sim`
- `drone_agent_real`

同时安装外层脚本：

- `scripts/camera_view_sim`

`launch/`

存放 ROS2 launch 文件。它们属于 ROS2 工程接入层，不属于 Agent 本体逻辑。

`scripts/`

存放 ROS2 辅助脚本，包括：

- `camera_view_sim`：相机画面查看脚本
- `drone_agent_sim`：直接启动仿真模式的命令包装脚本
- `drone_agent_real`：直接启动真机模式的命令包装脚本

这些脚本不放在内层 `drone_agent/` 包里。其中 `drone_agent_sim` 和 `drone_agent_real` 是独立的 Python 包装脚本，与 `setup.py` 中通过 `console_scripts` 定义的入口功能相同，但可以直接作为脚本文件执行。

### 6.2 内层 Python 包

#### `drone_agent/cli.py`

负责命令入口到 runtime 的衔接。

当前主要职责：

- 提供 `main_sim()`
- 提供 `main_real()`
- 捕获配置错误并输出到 stderr

它不负责 ROS2 初始化，也不负责 Agent 对话循环。

#### `drone_agent/config/`

负责 profile 和模型设置加载。

`schema.py`

定义当前真实使用的配置结构：

- `RosConfig`
- `StorageConfig`
- `ProviderConfig`
- `VlmConfig`
- `SafetyConfig`
- `RuntimeProfile`

`SafetyConfig` 当前字段是：

- `human_in_the_loop_for_flight_tools`
- `max_takeoff_height_m`
- `max_relative_move_m`
- `max_vertical_move_m`
- `max_rotation_deg`
- `action_timeout_s`
- `hover_on_timeout`
- `pre_takeoff_gate_enabled`
- `require_battery_status_for_takeoff`
- `min_battery_percent_for_takeoff`
- `require_px4_status_ready_for_takeoff`

`loader.py`

负责从两类来源组装运行时配置：

1. profile yaml
2. `settings.json`

当前不再使用环境变量直接注入 LLM/VLM 的 `api_key`、`base_url`、`model`。

默认设置文件位置：

```text
~/.config/drone_agent/settings.json
```

也可以通过环境变量覆盖：

```text
DRONE_AGENT_SETTINGS
```

`profiles/sim.yaml` 和 `profiles/real.yaml`

只负责：

- ROS 节点名
- 相机 topic
- 图片保存目录
- 日志目录
- 安全限制

模型提供方配置已经移出 YAML，不再放在 profile 里。

#### `drone_agent/llm/`

`client.py`

根据 `RuntimeProfile.llm` 创建 OpenAI 兼容文本模型 client。

`prompts.py`

存放文本 Agent 的系统提示词 `SYSTEM_PROMPT`。

#### `drone_agent/runtime/`

这是当前真正的运行时层，原先的 `core/` 已经重命名为 `runtime/`。

`runtime.py`

负责：

- 提供 `prepare_runtime(profile_name)` 函数：只加载并校验 profile，不启动 ROS2，返回 `RuntimeStartResult` 数据类，供测试和诊断使用
- 提供 `RuntimeStartResult` 数据类：保存 profile 解析后的运行时摘要（`profile_name`、`mode`、`node_name`、`ros_started`）
- 提供 `start_runtime(profile_name)` 函数：加载 profile 并直接启动真实 ROS2 运行时
- 初始化 ROS2
- 创建 `Px4Controller`
- 创建 LLM client
- 创建 `TaskState` 实例
- 创建 `MessageBus` 实例
- 创建 `InputServer` 实例
- 创建 `ToolContext`（含 `task_state`）
- 生成 `session_id`
- 自动打开独立输入终端
- 从 `MessageBus` 消费用户输入并启动 agent loop
- 每轮用户输入后调用 `task_state.start_new_goal()`
- 每轮 agent loop 前后打印并记录任务状态
- 配置 readline，修复中文输入删除问题
- 退出时关闭 executor 和 ROS2

当前默认交互模式是“双终端”：

- 输出终端：显示 agent / tool / state / ROS2 日志
- 输入终端：负责 `you>` 自然语言输入和 HITL 的 `Y/N`

如果当前环境无法自动打开新终端，则回退为旧的单终端输入线程模式。

`agent_loop.py`

只负责单轮 Agent 循环：

- 把 messages 发给 LLM
- 接收 tool calls
- 调用 dispatcher
- 把 tool result 追加回 messages
- 得到最终 assistant 回复
- 在每轮 LLM 调用前后更新 `TaskState`（`set_thinking` / `set_idle`）
- 打印并记录任务状态到日志

它不再负责终端输入循环。

`tool_dispatcher.py`

负责：

- 解析模型返回的工具名和 JSON 参数
- 做工具存在性检查
- 执行 human in the loop 硬确认
- 在工具执行前检查是否已有用户介入消息
- 调用具体工具函数
- 记录工具调用日志
- 在超时、用户介入等硬中断场景结束当前轮
- 通过 `_update_task_state()` 在工具生命周期各阶段同步更新 `TaskState`：
  - `waiting_for_confirmation`：等待人工确认
  - `tool_running`：工具执行中
  - `tool_finished`：工具完成
  - `interrupted`：被中断

`task_state.py`

定义当前会话的最小运行时任务状态 `TaskState`。

核心字段：

- `task_id`：会话 ID
- `current_user_goal`：当前用户目标
- `current_phase`：执行阶段（`idle` / `thinking` / `tool_running` / `tool_completed` / `tool_failed` / `interrupted` / `waiting_for_confirmation`）
- `active_tool_name`：当前正在执行的工具名
- `active_tool_arguments`：当前工具参数
- `active_tool_is_flight_tool`：当前工具是否为飞行控制工具
- `active_agent_name`：当前 agent 名称（默认 `drone_agent`）
- `waiting_for_user_confirmation`：是否正在等待人工确认
- `intervention_pending`：是否有待处理的用户介入
- `intervention_message`：介入消息内容
- `last_tool_name`：上一个执行的工具名
- `last_tool_result`：上一个工具的执行结果
- `last_error`：上一个错误信息

状态转移方法：

- `start_new_goal(user_input)`：用户输入新任务后刷新状态
- `set_thinking()`：标记进入模型思考阶段
- `set_idle()`：标记回到空闲状态
- `set_waiting_for_confirmation(tool_name, arguments, is_flight_tool)`：标记等待人工确认
- `start_tool(tool_name, arguments, is_flight_tool)`：标记工具开始执行
- `finish_tool(tool_name, result)`：根据工具结果更新成功或失败
- `interrupt(tool_name, result)`：标记因拒绝、超时等原因被中断
- `mark_intervention(message)`：记录一条等待处理的用户介入消息
- `clear_intervention()`：清空已经交给 LLM 处理的介入状态
- `snapshot()`：导出当前状态快照，供日志记录使用

#### `drone_agent/bus/`

`bus/` 是 Phase 6 新增的运行时消息总线层。

`queue.py`

封装线程安全的同步消息队列 `SyncMessageQueue`，提供：

- `publish()`
- `consume()`
- `try_consume()`
- `has_pending()`

`message_bus.py`

定义：

- `UserMessage`
- `MessageBus`

当前主要用于把用户自然语言输入从独立输入终端或回退输入线程传递给 runtime、dispatcher 和工具函数。

`input_server.py`

定义：

- `InputServerInfo`
- `InputServer`

负责在主进程中接收独立输入终端发送的用户消息。

`input_terminal.py`

负责运行在新终端中的输入客户端：

- 显示 `you>`
- 读取自然语言或 `Y/N`
- 通过本地 socket 回传主进程

`intervention.py`

集中处理语言介入：

- `should_interrupt(context)`：判断是否存在待处理用户介入
- `consume_intervention(context)`：取出介入消息并更新 `TaskState`
- `build_interrupted_result(...)`：构造 `INTERRUPTED_BY_USER` 结果
- `interrupt_if_requested(...)`：统一检测介入，必要时对飞行工具执行 hover 保护动作

终端输出：

- `format_task_state_line()`：格式化带颜色的终端状态行，绿色显示当前阶段、活跃工具名、是否飞行工具、是否等待确认、错误信息

`safety.py`

负责当前 Agent 侧安全规则：

- `FLIGHT_TOOL_NAMES`
- `requires_human_in_the_loop()`
- `confirm_flight_tool()`
- `should_end_turn_after_tool_result()`
- `EndCurrentTurn`

当前规则是：

- 只要开启 `human_in_the_loop_for_flight_tools`
- 且工具名属于 `FLIGHT_TOOL_NAMES`
- 就在工具执行前强制 Y/N 人工确认

Phase 7 当前已落地的最小扩展是：

- `takeoff()` 内增加真机起飞前检查
- 可检查 PX4 状态是否已收到
- 可检查电池状态是否已收到
- 可检查电量是否高于起飞阈值
- 检查失败时直接拒绝起飞

输入：

- `Y`：执行工具
- `N`：立即结束当前轮，把控制权还给用户

此外，当前把 `*_TIMEOUT` 也视为硬中断，工具超时后不会继续让 LLM 串行执行下一步飞行动作。

#### `drone_agent/px4/`

`controller.py`

当前底层 PX4 控制器，直接继承 `rclpy.node.Node`。

负责：

- 创建 PX4 publishers/subscribers
- 订阅本地位置、飞控状态、电池状态
- 可选订阅相机 topic
- 缓存最新 RGB 画面
- 发布 offboard 心跳
- 发布 setpoint
- 发布 vehicle command
- 维护位置保持、目标航向和目标到达判断

这是直接依赖 ROS2 / px4_msgs 的实现，没有为了测试额外引入依赖注入抽象。

`frame.py`

坐标转换工具，例如 body FRD 到 world NED。

`status.py`

把 PX4 状态枚举整理成可读信息。

`topics.py`

集中定义 PX4 topic 常量。

#### `drone_agent/tools/`

这是暴露给 LLM Function Calling 的工具层。

`registry.py`

定义：

- `ToolContext`
- `ToolDefinition`
- 工具名到处理函数的映射

当前 `ToolContext` 包含：

- `controller`
- `profile`
- `session_id`
- `task_state`（`TaskState | None`，可选）

`schemas.py`

集中存放所有工具 schema。

`flight.py`

当前飞行动作工具：

- `takeoff`
- `land`
- `disarm`
- `timer`
- `hover`
- `return_home`
- `rotate`
- `move`

它们负责动作级业务逻辑和结果结构化返回。

当前不再通过工具返回字段让 LLM 判断是否暂停或继续。

超时时通常返回：

- `success: false`
- `error: *_TIMEOUT`
- `message`
- `target_position_ned`
- `final_position_ned`

然后由 `runtime/tool_dispatcher.py` 决定是否直接结束当前轮。

`status.py`

当前状态工具：

- `current_position_status`
- `battery_status`
- `flight_mode_status`

`perception.py`

当前感知工具：

- `take_photo`
- `analyze_view`

它依赖当前 controller 缓存的最新 RGB 图像，以及 `vision/` 模块。

#### `drone_agent/vision/`

`image_store.py`

负责图片落盘：

- 拍照结果保存
- 分析帧保存

`prompts.py`

存放 VLM 的系统提示词 `VLM_SYSTEM_PROMPT`。

`vlm.py`

负责：

- 创建视觉模型 client
- 构造分析 prompt
- 本地图像编码为 data URL
- 调用视觉模型
- 提取 JSON
- 归一化目标位置、偏移和置信度
- 推导视觉建议动作

当前没有单独的 `camera.py` 文件，因为相机图像接收和缓存直接放在 `Px4Controller` 中。

#### `drone_agent/logging/`

`task_log.py`

负责记录三类 JSONL 日志：

- `agent_messages.jsonl`：Agent 对话消息
- `tool_calls.jsonl`：工具调用及其结果
- `task_state.jsonl`：任务状态快照

新增 `log_task_state()` 函数，每次 `TaskState` 状态转移时记录快照，包含：

- `timestamp`
- `profile_name`
- `event_type: "task_state"`
- `task_id`、`current_phase`、`current_user_goal`
- `active_tool_name`、`active_tool_is_flight_tool`
- `waiting_for_user_confirmation`、`intervention_pending`
- `last_tool_name`、`last_error`

当前日志目录按会话拆分，结构是：

```text
/home/hw/drone_agent_logs/sim/
  session_20260613_210501/
    agent_messages.jsonl
    tool_calls.jsonl
    task_state.jsonl
```

时间戳使用东八区北京时间字符串，例如：

```text
2026-06-13 21:05:03
```

当前没有 `flight_log.py`，所有已实现日志都集中在 `task_log.py`。

## 7. 当前配置方式

### 7.1 profile yaml

`sim.yaml`

- `mode: simulation`
- `ros.node_name: drone_agent_sim`
- `ros.camera_scene_topic: /airsim_node/PX4/CameraDepth1/Scene`
- `storage.log_dir: /home/hw/drone_agent_logs/sim`
- `safety.human_in_the_loop_for_flight_tools: false`

`real.yaml`

- `mode: real`
- `ros.node_name: drone_agent_real`
- `ros.camera_scene_topic: null`
- `storage.log_dir: /home/hw/drone_agent_logs/real`
- `safety.human_in_the_loop_for_flight_tools: true`

### 7.2 settings.json

当前模型配置由 `settings.json` 提供。

示例结构：

```json
{
  "llm": {
    "api_key": "replace-with-your-llm-key",
    "base_url": "replace-with-your-llm-base-url",
    "model": "replace-with-your-llm-model"
  },
  "vlm": {
    "enabled": true,
    "api_key": "replace-with-your-vlm-key",
    "base_url": "replace-with-your-vlm-base-url",
    "model": "replace-with-your-vlm-model"
  }
}
```

结论是：

- `api_key` 自己填
- `base_url` 自己填
- `model` 自己填

项目不再提供默认 provider 参数。

## 8. 当前安全契约

### 8.1 human in the loop

当前 human in the loop 是**代码硬限制**，不是靠 LLM 理解某个返回字段来决定暂停还是继续。

触发条件：

- `profile.safety.human_in_the_loop_for_flight_tools == true`
- 工具名属于 `FLIGHT_TOOL_NAMES`

行为：

- 执行前提示 `Y/N`
- `Y` 继续执行
- `N` 直接结束当前轮，回到用户输入

### 8.2 超时处理

当前工具超时后：

1. 工具函数返回 `*_TIMEOUT`
2. `should_end_turn_after_tool_result()` 识别为硬中断
3. `tool_dispatcher.py` 抛出 `EndCurrentTurn`
4. `agent_loop.py` 结束本轮

因此，超时后不会继续串行调用后续飞行动作。

### 8.3 位置字段语义

当前仍保留这组重要语义：

- `target_position_ned`：目标位置，不代表已经到达
- `final_position_ned`：工具结束时的实际位置

## 9. 当前已实现工具集合

当前 Function Calling 工具集合是：

- `takeoff`
- `land`
- `disarm`
- `timer`
- `hover`
- `return_home`
- `current_position_status`
- `battery_status`
- `flight_mode_status`
- `rotate`
- `move`
- `take_photo`
- `analyze_view`

## 10. 当前已知边界

当前实现已经明确的边界包括：

- 不做 Web UI
- 不做多机协同
- 不把 AirSim 相机发布逻辑纳入本项目
- 不在内层 `drone_agent/` 中放 ROS2 辅助脚本

此外，当前还有几个重要“未实现但已知”的边界：

- 没有单独的 `vision/camera.py`
- 没有 `logging/flight_log.py`
- 没有 `runtime/types.py`
- 已有说明型 `skills/` 第一版，但没有可执行 skill 脚本和自进化生成链路
- 没有 async runtime，当前语言介入仍基于同步 `MessageBus`、独立输入终端或输入线程回退模式、以及工具内部检查点
- 没有异步 multi-agent 架构

这些都不是遗漏，而是当前版本尚未引入。

## 11. 后续规划方向

本节只记录后续方向，不代表当前已经实现。下一阶段如果开始落地，应为每个方向单独写设计文档和实施计划。

### 11.1 Claude/Codex 风格 skills 与自进化能力

这里的 `skills` 不是“把多个 tools 顺序封装成一个复合工具”。后续要引入的 `skills` 更接近 Claude / Codex 的 Skill：它是一套可复用的能力说明、工作流、约束、模板、示例和可选脚本，用来指导 Agent 在特定任务域中更稳定地工作。

建议边界：

- `tools/`：LLM Function Calling 可直接调用的原子动作，例如 `takeoff`、`move`、`analyze_view`。
- `skills/`：Agent 可读取和遵循的任务方法论，例如“视觉搜索目标”“真机低高度试飞检查”“日志复盘生成候选经验”。
- `agents/`：后期 multi-agent 中不同职责的执行体，例如 planner、vision agent、motor agent。

未来 `skills/` 可以采用下面的结构：

```text
drone_agent/
  skills/
    visual_search/
      SKILL.md
      examples/
      scripts/
    real_flight_check/
      SKILL.md
      examples/
    log_to_skill_candidate/
      SKILL.md
      templates/
```

`SKILL.md` 建议包含：

- skill 名称和适用场景。
- 触发条件：什么用户请求或什么日志模式应该使用它。
- 工作流：Agent 应该按什么步骤思考和执行。
- 可调用 tools：允许使用哪些工具。
- 安全约束：哪些动作必须确认，哪些情况必须停止。
- 失败处理：工具失败、超时、低置信度时如何处理。
- 反例：哪些情况不应该使用该 skill。
- 示例：典型用户请求、工具调用轨迹、期望结果。

自进化流程建议：

1. Agent 从会话日志中发现重复任务模式或高价值经验。
2. Agent 调用内置 `skill creator`，读取相关 `agent_messages.jsonl` 和 `tool_calls.jsonl`。
3. `skill creator` 生成候选 `SKILL.md`。
4. 用户 review 候选 skill。
5. 用户确认后，才写入 `drone_agent/skills/<skill_name>/SKILL.md`。
6. 新 skill 默认先进入 `draft` 或 `disabled` 状态。
7. 用户显式启用后，Agent 才能在后续任务中使用。

关键原则：

- 自动提炼可以由 Agent 做。
- 正式生成必须由用户确认。
- 初期只生成说明型/流程型 `SKILL.md`，不自动生成可执行 Python 代码。
- 如果后期允许生成脚本，脚本必须经过用户 review，并默认只作为辅助工具，不直接绕过飞控安全层。

### 11.2 真机安全门现状与后续补充

当前已经实现了一部分基础安全门：

- `human_in_the_loop_for_flight_tools`：飞行工具执行前 Y/N 硬确认。
- `max_takeoff_height_m`：最大起飞高度限制。
- `max_relative_move_m`：单次水平移动距离限制。
- `max_vertical_move_m`：单次垂直移动距离限制。
- `max_rotation_deg`：单次旋转角度限制。
- `action_timeout_s`：动作超时限制。
- `hover_on_timeout`：超时后进入 hover。
- 本地位置有效性检查。
- 工具返回 `*_TIMEOUT` 后结束当前轮，不继续串行执行后续飞行动作。

后续真机试验需要补充更细粒度安全门。

这些安全门是否启用、阈值是多少，后续应继续放在 `drone_agent/config/profiles/*.yaml` 中由用户选择。仿真和真机可以使用同一套字段，但默认值不同。

示例扩展：

```yaml
safety:
  preflight_checks_enabled: true
  require_battery_check: true
  min_battery_percent: 30
  require_position_valid_before_arm: true
  require_camera_ready_before_vision: true
  detect_other_fmu_publishers: true
  stop_on_position_deviation: true
  max_position_deviation_m: 0.8
  block_large_motion_on_low_vlm_confidence: true
  min_vlm_confidence_for_motion: 0.6
  user_intervention_hover_for_flight_tools: true
```

#### 启动前安全门

- ROS2/PX4 DDS 链路是否正常。
- PX4 本地位置是否有效。
- 飞控状态是否可读。
- 电池是否连接。
- 电池电量是否高于真机阈值。
- 是否检测到相机 topic。
- 是否有其它控制节点在发布 `/fmu/in/*`。

#### 解锁前安全门

- 用户是否明确确认当前是真机运行。
- 是否处于允许解锁的飞行状态。
- 是否满足 position lock。
- 是否满足最低电量。
- 是否允许当前任务进入解锁阶段。

#### 起飞安全门

- 起飞高度不得超过 profile 限制。
- 起飞前必须确认当前位置有效。
- 起飞后必须检查是否真的离地。
- 起飞失败必须进入 hover 或降落策略。

#### 移动安全门

- 单次水平移动距离限制。
- 单次垂直移动距离限制。
- 最大高度限制。
- 最大下降幅度限制。
- 移动前后检查当前位置偏差。
- 如果偏差过大，停止后续飞行动作。

#### 旋转安全门

- 单次旋转角度限制。
- 旋转超时后结束当前轮。
- 旋转后检查 yaw 是否接近目标。

#### 感知安全门

- 图像未就绪时不执行视觉任务。
- VLM 低置信度时不允许直接触发大幅移动。
- 视觉建议动作必须经过飞行安全限制二次校验。

#### 运行中安全门

- 低电量触发返航或降落建议。
- PX4 状态异常时停止后续飞行动作。
- 用户介入时立即 hover。
- 手动接管或遥控器优先级高于 Agent。

### 11.3 运行时任务状态（Phase 5 已完成）

**当前状态：已实现。**

`runtime/task_state.py` 已落地，包含以下能力：

已实现字段：

- `task_id`：会话 ID
- `current_user_goal`：当前用户目标
- `current_phase`：执行阶段
- `active_tool_name`：当前活跃工具名
- `active_tool_arguments`：当前工具参数
- `active_tool_is_flight_tool`：是否飞行控制工具
- `active_agent_name`：当前 agent 名称
- `waiting_for_user_confirmation`：是否等待人工确认
- `intervention_pending`：是否有待处理介入
- `intervention_message`：介入消息内容
- `last_tool_name`：上一个工具名
- `last_tool_result`：上一个工具结果
- `last_error`：上一个错误信息

已实现状态转移方法：

- `start_new_goal()`
- `set_thinking()`
- `set_idle()`
- `set_waiting_for_confirmation()`
- `start_tool()`
- `finish_tool()`
- `interrupt()`
- `snapshot()`

已实现终端输出：

- `format_task_state_line()`：带颜色的终端状态行

已实现日志集成：

- `task_log.log_task_state()`：每次状态转移记录到 `task_state.jsonl`

集成点：

- `runtime.py`：创建 `TaskState` 实例，传入 `ToolContext`
- `agent_loop.py`：每轮 LLM 调用前后更新状态
- `tool_dispatcher.py`：工具生命周期各阶段同步更新状态

后续扩展方向（当前未实现）：

- 多 agent 场景下的 `active_agent_name`、`active_flight_action`、`last_vision_summary`、`last_motor_summary` 等字段。
- CLI 或 Web 界面中实时展示任务状态。

### 11.4 MessageBus 与语言介入

当前第一版语言介入已经落地，用户可以在独立输入终端持续输入自然语言打断正在执行的动作。这解决了单终端输入被日志刷屏干扰的问题。

当前已经引入一个中心 `MessageBus` 队列。后续 Planner、MotorAgent、VisionAgent 也可以继续通过这个 bus 传递消息。

基础结构可以是：

```text
用户输入线程 / async task
  -> MessageBus
  -> PlannerAgent
  -> MotorAgent / VisionAgent
```

语言介入的核心规则：

- 用户输入统一进入 `MessageBus`。
- 只要 Agent 正在执行任意工具，且 bus 中出现新的用户输入，则认为发生用户介入。
- 一旦发生用户介入，当前工具必须暂停或取消。
- 如果当前工具是飞行控制相关工具，飞控层必须立即进入 hover。
- 如果当前工具不是飞行控制相关工具，不需要 hover，但也必须停止当前工具执行，等待介入语句处理完成。
- 介入消息被补充给 LLM / Planner。
- Planner 处理介入语句后，再决定继续、修改任务、返航、降落或取消任务。
- 如果同一条 assistant message 中存在多个 `tool_calls`，当前轮因为介入提前结束时，`agent_loop.py` 必须给剩余未执行的 `tool_call_id` 补 `SKIPPED_DUE_TO_TURN_END` 结果，保持下一轮请求的消息协议合法。

当前实现边界：

- `MessageBus` 可以排队保存多条用户消息。
- 一次介入中断只会先消费一条消息。
- 剩余消息继续保留在队列中，后续再消费。

介入流程建议：

```text
Agent 正在执行任意工具
  -> MessageBus 收到新的 UserMessage
  -> runtime 标记 intervention_pending=true
  -> 当前工具停止执行并返回 INTERRUPTED_BY_USER
  -> 如果当前工具属于飞行控制相关工具，MotorAgent 立即调用 hover / emergency_hover
  -> PlannerAgent 读取用户介入消息
  -> PlannerAgent 重新规划或请求用户确认
```

最小消息类型建议：

- `UserMessage`
- `UserIntervention`
- `PlannerMessage`
- `TaskAssignment`
- `TaskResult`
- `SafetyCommand`

不需要一开始定义大量事件类型。后续如果确实需要，再按实际行为扩展。

### 11.5 异步 multi-agent 架构

后期引入 multi-agent 时，建议从当前单体同步 loop 演进到异步 message bus 架构。

目标角色：

- `PlannerAgent`：理解用户目标，生成计划，决定任务分派。
- `VisionAgent`：负责相机画面、VLM、目标识别、场景总结。
- `MotorAgent`：负责飞行动作、PX4 控制、安全执行。

推荐架构：

```text
Runtime
  -> MessageBus
  -> TaskState
  -> PlannerAgent
  -> VisionAgent
  -> MotorAgent
  -> Px4Controller
```

推荐运行模型：

- 使用 `asyncio` 作为主并发模型。
- 用户输入监听、PlannerAgent、VisionAgent、MotorAgent、日志记录分别作为 async task。
- Agent 之间不直接互相调用，而是通过 `MessageBus` 传递任务和结果。
- `MotorAgent` 拥有唯一飞控执行权。
- `PlannerAgent` 只能向 `MotorAgent` 发任务，不能直接绕过 `MotorAgent` 控制 PX4。
- `VisionAgent` 只返回精简视觉摘要，不把完整视觉对话上下文塞回 Planner。
- `MotorAgent` 只返回精简动作结果和飞行状态摘要，不把每一步内部执行细节塞回 Planner。

Planner 工作流建议：

1. 用户提出任务。
2. `PlannerAgent` 生成计划。
3. 用户查看并确认计划。
4. `PlannerAgent` 把飞控任务分配给 `MotorAgent`。
5. `PlannerAgent` 把视觉任务分配给 `VisionAgent`。
6. `MotorAgent` 和 `VisionAgent` 在不冲突时可以并行执行。
7. 两个 agent 返回有效精简上下文。
8. `PlannerAgent` 根据结果决定下一步或结束任务。

CLI 或 Web 界面后续应展示 Planner 的计划和分派情况。最小可见信息包括：

- 当前用户目标。
- Planner 生成的步骤列表。
- 每个步骤分配给哪个 agent。
- 每个步骤状态：`pending`、`running`、`completed`、`failed`、`interrupted`。
- VisionAgent 返回的场景摘要、目标是否出现、置信度。
- MotorAgent 返回的动作摘要、最终位置、电池和飞控状态。
- 当前是否需要用户确认计划、确认飞行动作或处理介入消息。

这种结构的价值：

- 飞行动作和视觉观察可以并行，提高效率。
- Planner 的上下文更干净，只接收必要摘要。
- 飞控执行权集中在 MotorAgent，安全边界清楚。
- 用户介入可以通过 MessageBus 统一处理。
- 后续扩展更多 agent 时，不需要让所有 agent 互相直接依赖。

### 11.6 后续实施顺序建议

后续实现应收敛为 5 个大方向，并按下面顺序推进：

```text
✅ TaskState（已完成） -> ✅ 语言介入（已完成第一版） -> 安全门 -> Skills -> Multi-Agent
```

这个顺序的判断依据是：先让系统知道自己正在做什么，再让用户可以随时介入，然后补真机安全边界，最后再做自进化能力和多 agent 协作。无人机 Agent 的核心风险不是能力不足，而是执行过程不可观测、不可打断、不可解释。

#### Phase 5：TaskState（已完成）

**当前状态：已完成，详见 `runtime/task_state.py`。**

完成内容：

- ✅ 记录当前用户目标。
- ✅ 记录当前正在执行的工具。
- ✅ 标记当前工具是否属于飞行控制工具。
- ✅ 记录工具执行状态：`idle`、`thinking`、`tool_running`、`tool_completed`、`tool_failed`、`interrupted`、`waiting_for_confirmation`。
- ✅ 记录最近一次工具结果。
- ✅ 记录是否等待用户确认。
- ✅ 记录是否存在用户介入消息。
- ✅ 终端彩色状态输出。
- ✅ 日志持久化到 `task_state.jsonl`。
- ✅ 集成到 `runtime.py`、`agent_loop.py`、`tool_dispatcher.py`。

设计文档：`docs/PHASE_5_TASK_STATE_DESIGN.md`

#### Phase 6：语言介入（已完成第一版）

目标：解决 Agent 执行工具期间用户无法打断的问题。

前置依赖：Phase 5 TaskState 已完成，`intervention_pending` 字段已就绪。

完成内容：

- ✅ 新增 `drone_agent/bus/queue.py`，封装同步消息队列。
- ✅ 新增 `drone_agent/bus/message_bus.py`，提供用户消息发布与消费。
- ✅ 新增 `drone_agent/bus/input_server.py`，接收独立输入终端消息。
- ✅ 新增 `drone_agent/bus/input_terminal.py`，作为独立输入终端客户端。
- ✅ 新增 `drone_agent/bus/intervention.py`，集中处理用户介入检测与中断结果。
- ✅ 用户输入默认由独立输入终端写入 `MessageBus`。
- ✅ runtime 主循环从 `MessageBus` 消费用户消息。
- ✅ `ToolContext` 已携带 `message_bus`。
- ✅ `tool_dispatcher.py` 在工具执行前检查介入消息。
- ✅ `timer()` 支持非飞行工具介入中断。
- ✅ `takeoff()`、`move()`、`rotate()`、`land()` 支持飞行工具介入中断。
- ✅ 飞行工具介入时先发送 hover 保护动作。
- ✅ `INTERRUPTED_BY_USER` 会结束当前轮，不继续旧任务链路。
- ✅ 介入消息会作为新的用户消息进入下一轮 LLM。
- ✅ 输出终端与输入终端分离，降低日志对刷输入的干扰。

设计文档：`docs/PHASE_6_LANGUAGE_INTERVENTION_DESIGN.md`

#### Phase 7：起飞前安全门（已完成最小版）

目标：只在 `takeoff()` 内补少量真机起飞前检查，不新增外层安全门系统。

完成内容：

- ✅ `SafetyConfig` 新增 `pre_takeoff_gate_enabled`
- ✅ `SafetyConfig` 新增 `require_battery_status_for_takeoff`
- ✅ `SafetyConfig` 新增 `min_battery_percent_for_takeoff`
- ✅ `SafetyConfig` 新增 `require_px4_status_ready_for_takeoff`
- ✅ `real.yaml` 默认启用最小起飞前检查
- ✅ `sim.yaml` 默认关闭最小起飞前检查
- ✅ `Px4Controller` 记录 `vehicle_status_received`
- ✅ `Px4Controller` 记录 `battery_status_received`
- ✅ `takeoff()` 在现有参数检查后增加真机起飞前检查
- ✅ 电池状态不可读时拒绝起飞
- ✅ 电量低于阈值时拒绝起飞
- ✅ PX4 状态不可读时拒绝起飞

设计文档：`docs/PHASE_7_SAFETY_GATES_DESIGN.md`

当前边界：

- LLM API 请求本身仍不能被同步队列即时取消。
- 阻塞式 VLM/API 调用如果没有检查点，仍需要等调用返回后才能处理介入。
- 完整 async runtime 和 `asyncio.Queue` 留到后续 multi-agent 阶段统一设计。

#### Phase 7：安全门

目标：把当前基础安全限制扩展成真机试验级安全门。

范围：

- 安全门开关和阈值继续放在 `config/profiles/*.yaml`。
- 先补启动前检查、解锁前检查、电池阈值、位置有效性、动作偏差检查。
- 再补 VLM 低置信度动作限制、PX4 状态异常处理、其它控制节点冲突检测。

建议顺序：

1. 先扩展 profile schema。
2. 再实现启动前检查。
3. 再实现解锁前检查。
4. 再实现飞行动作执行前后的偏差检查。
5. 最后再加感知相关安全门。

验收标准：

- 仿真和真机可以使用不同默认安全策略。
- 用户能在 profile 中选择启用或关闭具体安全门。
- 真机 profile 默认比仿真 profile 更保守。

#### Phase 8：Skills（已完成第一版）

目标：引入 Claude/Codex 风格的 `SKILL.md` 能力包，让 Agent 可以复用经过确认的任务经验和流程。

范围：

- Phase 8 第一版只支持项目内置 `drone_agent/skills/` 中的手写 skill。
- 所有已启用 skill 的 `name + description` 会组成全局 `skills index`。
- 主 LLM 根据 `skills index` 判断是否需要 skill，并通过 `activate_skill(name)` 启用。
- `activate_skill` 通过 Python 校验、Y/N 人工确认后，将选中的 skill 正文作为工具结果返回给主 LLM。
- skill 只影响 Agent 的规划和工具选择，不直接生成飞控代码。
- skill 不能绕过 `tools/`、HITL、语言介入、超时和起飞前安全门。
- 需要提供 `skill_creator`，用于人工创建和校验标准格式的手写 skill。
- 第一版不支持 `scripts/`、workspace 覆盖、打包分发或从日志自动生成 skill。

验收标准：

- Agent 能加载项目内置 `SKILL.md`。
- Agent 能构建全局 `skills index`。
- `skill_creator` 能生成标准格式的 `SKILL.md` 草稿。
- 主 LLM 能通过 `activate_skill` 启用 `visual-search` 或 `real-low-altitude-test`。
- `activate_skill` 会校验 skill 是否存在、是否启用、当前 profile 是否允许。
- 启用 skill 前会触发 Y/N 人工确认。
- 用户确认后，skill 正文通过工具结果返回给主 LLM。
- 日志通过 `tool_calls.jsonl` 记录 skill 启用。
- skill 不改变 tool schema，也不新增飞控执行入口。

当前实现文件：

- `drone_agent/skills/validator.py`
- `drone_agent/skills/skill_creator.py`
- `drone_agent/skills/loader.py`
- `drone_agent/skills/context.py`
- `drone_agent/skills/visual-search/SKILL.md`
- `drone_agent/skills/real-low-altitude-test/SKILL.md`
- `drone_agent/tools/skill.py`
- `runtime.py` 构建 `skills index`
- `agent_loop.py` 通过普通工具调用接收 `activate_skill` 结果

设计文档：`docs/PHASE_8_SKILLS_DESIGN.md`

#### Phase 9：Multi-Agent

目标：从单 Agent loop 演进到 Planner / Vision / Motor 分工架构。

范围：

- `PlannerAgent` 先生成计划，并给用户确认。
- `PlannerAgent` 将视觉任务分给 `VisionAgent`。
- `PlannerAgent` 将飞控任务分给 `MotorAgent`。
- `VisionAgent` 返回精简视觉摘要。
- `MotorAgent` 返回精简动作结果和飞行状态摘要。
- `MotorAgent` 是唯一飞控执行出口。

第一版 multi-agent 不应追求复杂并发。可以先实现串行分派和结果汇总，再逐步允许 VisionAgent 和 MotorAgent 在不冲突的任务上并行执行。

验收标准：

- 用户能看到 Planner 的计划并确认。
- 用户能看到任务分配给了哪个 agent。
- Planner 上下文只接收必要摘要，不接收所有底层工具细节。
- 两个飞行动作不会并行执行。

核心原则：

- 真机安全优先级高于任务完成。
- 用户介入优先级高于 Planner。
- 任意工具执行期间都应响应用户介入。
- 飞行控制相关工具被介入时必须先 hover。
- `MotorAgent` 或等价飞控执行层必须是唯一飞行控制出口。
- Planner 的计划必须先给用户确认。
- 自动生成 skill 必须经过用户确认。
- 新能力先在仿真验证，再进入真机 profile。

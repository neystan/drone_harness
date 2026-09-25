# Phase 5：TaskState 设计文档

日期：2026-06-14

## 1. 目标

Phase 5 的目标是引入最小可用的运行时任务状态 `TaskState`。

它解决的问题不是长期记忆，也不是历史任务恢复，而是让当前正在运行的 agent 清楚知道：

- 当前用户目标是什么。
- 当前是否正在执行工具。
- 当前执行的工具是什么。
- 当前工具是否属于飞行控制工具。
- 当前是否等待用户确认。
- 当前是否有用户介入消息等待处理。
- 最近一次工具调用结果是什么。

这个状态会成为后续“语言介入”“安全门”“multi-agent”的基础。

## 2. 不做什么

Phase 5 不做：

- 长期记忆。
- 历史任务恢复。
- 数据库。
- Web UI。
- MessageBus。
- multi-agent。
- 自动生成 skills。

Phase 5 只补状态对象和状态更新点，保持当前同步 CLI 架构不变。

## 3. 设计原则

`TaskState` 必须简单、直接、可读。

当前项目不需要复杂状态机框架，也不需要为了测试引入额外抽象。状态对象应该是普通 dataclass，上层 runtime 和 dispatcher 直接读写。

核心原则：

- 工具执行前更新状态。
- 工具执行后更新状态。
- 工具中断或超时也必须更新状态。
- 状态可以被日志记录。
- 状态可以被 CLI 打印。
- 后续 MessageBus 和 multi-agent 可以复用它。

## 4. 建议文件结构

新增文件：

```text
drone_agent/
  runtime/
    task_state.py
```

相关改动文件：

```text
drone_agent/
  runtime/
    runtime.py
    agent_loop.py
    tool_dispatcher.py

  tools/
    registry.py

  logging/
    task_log.py
```

## 5. TaskState 字段

建议第一版字段如下：

```python
@dataclass
class TaskState:
    task_id: str
    current_user_goal: str | None = None
    current_phase: str = "idle"
    active_tool_name: str | None = None
    active_tool_arguments: dict[str, Any] | None = None
    active_tool_is_flight_tool: bool = False
    active_agent_name: str = "drone_agent"
    waiting_for_user_confirmation: bool = False
    intervention_pending: bool = False
    intervention_message: str | None = None
    last_tool_name: str | None = None
    last_tool_result: dict[str, Any] | None = None
    last_error: str | None = None
```

`current_phase` 第一版只需要这几个值：

- `idle`
- `thinking`
- `waiting_for_confirmation`
- `tool_running`
- `tool_completed`
- `tool_failed`
- `interrupted`

这些值不需要先做成复杂 enum。可以先用字符串常量，后续状态变复杂时再收敛。

## 6. 状态更新时机

### 6.1 用户输入后

位置：`runtime.py`

当用户输入新任务后：

- `current_user_goal = user_input`
- `current_phase = "thinking"`
- 清空上一轮 `active_tool_name`
- 清空上一轮 `active_tool_arguments`
- 清空上一轮 `last_error`

### 6.2 LLM 返回 tool call 前后

位置：`agent_loop.py`

当进入模型推理阶段：

- `current_phase = "thinking"`

当模型返回最终文本：

- `current_phase = "idle"`

### 6.3 工具执行前

位置：`tool_dispatcher.py`

在真正调用工具 handler 前：

- `current_phase = "tool_running"`
- `active_tool_name = tool_name`
- `active_tool_arguments = arguments`
- `active_tool_is_flight_tool = tool_name in FLIGHT_TOOL_NAMES`
- `waiting_for_user_confirmation = False`

如果该工具需要 human in the loop：

- `current_phase = "waiting_for_confirmation"`
- `waiting_for_user_confirmation = True`

用户确认后再进入：

- `current_phase = "tool_running"`
- `waiting_for_user_confirmation = False`

### 6.4 工具执行成功后

位置：`tool_dispatcher.py`

工具返回 `success=True` 后：

- `current_phase = "tool_completed"`
- `last_tool_name = tool_name`
- `last_tool_result = result`
- `last_error = None`
- 清空 `active_tool_name`
- 清空 `active_tool_arguments`
- `active_tool_is_flight_tool = False`

### 6.5 工具失败或超时后

位置：`tool_dispatcher.py`

工具返回 `success=False` 后：

- `current_phase = "tool_failed"`
- `last_tool_name = tool_name`
- `last_tool_result = result`
- `last_error = result.get("error")`

如果错误导致当前轮结束，例如 `*_TIMEOUT`：

- `current_phase = "interrupted"`
- 清空 active tool 字段

### 6.6 用户拒绝 human in the loop 后

位置：`tool_dispatcher.py`

如果用户输入 `N`：

- `current_phase = "interrupted"`
- `last_tool_name = tool_name`
- `last_tool_result = HUMAN_IN_THE_LOOP_DECLINED`
- `last_error = "HUMAN_IN_THE_LOOP_DECLINED"`
- 清空 active tool 字段

## 7. ToolContext 扩展

当前 `ToolContext` 包含：

```python
controller
profile
session_id
```

Phase 5 建议扩展为：

```python
controller
profile
session_id
task_state
```

这样工具分发器和后续 tools 都能访问同一个运行时状态。

## 8. 日志设计

Phase 5 不需要新建复杂日志系统，但可以在当前会话日志中增加状态记录。

建议在 `task_log.py` 中新增：

```python
log_task_state(profile, session_id, task_state)
```

第一版可以写入：

```text
task_state.jsonl
```

会话目录结构变为：

```text
session_20260614_153000/
  agent_messages.jsonl
  tool_calls.jsonl
  task_state.jsonl
```

`task_state.jsonl` 每条记录包含：

- `timestamp`
- `profile_name`
- `event_type: task_state`
- `task_id`
- `current_phase`
- `current_user_goal`
- `active_tool_name`
- `active_tool_is_flight_tool`
- `waiting_for_user_confirmation`
- `intervention_pending`
- `last_tool_name`
- `last_error`

不要把完整大对象盲目写入日志，避免日志变得难读。

## 9. CLI 可见性

Phase 5 可以先做轻量 CLI 可见性。

建议在关键状态变化时打印：

```text
state> thinking
state> tool_running takeoff flight_tool=true
state> waiting_for_confirmation takeoff
state> tool_completed takeoff
state> interrupted takeoff error=TAKEOFF_TIMEOUT
```

这不是最终 UI，只是为了真机测试时让用户知道 agent 当前在做什么。

## 10. 与后续阶段的关系

### 10.1 语言介入

Phase 6 会依赖 `TaskState` 判断：

- 当前是否正在执行工具。
- 当前工具是否飞行工具。
- 是否需要 hover。
- 介入消息是否 pending。

### 10.2 安全门

Phase 7 会依赖 `TaskState` 记录：

- 安全门执行阶段。
- 当前是否允许解锁。
- 当前是否允许继续飞行动作。
- 最近一次安全检查失败原因。

### 10.3 Multi-Agent

Phase 9 会扩展 `TaskState`：

- `active_agent_name`
- `last_vision_summary`
- `last_motor_summary`
- plan step 状态

Phase 5 不需要提前实现这些字段，除非实现时发现加上字段更简单。

## 11. 验收标准

Phase 5 完成后应满足：

- 新增 `runtime/task_state.py`。
- `runtime.py` 创建一个会话级 `TaskState`。
- `ToolContext` 持有 `TaskState`。
- 用户输入后状态更新为 `thinking`。
- 工具执行前状态更新为 `tool_running`。
- 需要 human in the loop 时状态更新为 `waiting_for_confirmation`。
- 工具成功后状态更新为 `tool_completed`。
- 工具失败、超时、用户拒绝确认后状态更新为 `tool_failed` 或 `interrupted`。
- 会话日志中能看到 `task_state.jsonl`。
- CLI 中能看到最小状态提示。
- 不改变现有工具调用语义。
- 不改变 PX4 控制逻辑。

## 12. 实施建议

建议按下面顺序实现：

1. 新增 `runtime/task_state.py`。
2. 扩展 `ToolContext`。
3. 在 `runtime.py` 创建并传递 `TaskState`。
4. 在 `agent_loop.py` 更新 thinking / idle 状态。
5. 在 `tool_dispatcher.py` 更新工具执行状态。
6. 在 `task_log.py` 增加 `task_state.jsonl`。
7. 增加 CLI 状态打印。
8. 补本地测试。

这一步不要改成异步架构，也不要引入 MessageBus。Phase 5 的价值是把当前同步架构的状态先暴露出来。

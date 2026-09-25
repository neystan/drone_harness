# Phase 6：语言介入设计文档

日期：2026-06-18

## 1. 目标

Phase 6 的目标是让用户在 agent 执行工具期间仍然可以输入新的自然语言指令，并让系统及时中断当前工具执行。

这解决的不是 multi-agent，也不是完整异步任务编排，而是真机测试中的一个关键安全问题：

- agent 正在执行工具时，用户不能只能等待工具结束。
- 用户一旦输入新的指令，系统应该识别为介入。
- 如果当前工具是飞行控制工具，无人机应立即进入 hover。
- 当前工具必须停止，旧任务链路中的后续工具不能继续自动执行。
- 介入消息要补充给 LLM，让 agent 根据新的用户意图重新判断下一步。

## 2. 不做什么

Phase 6 不做：

- 不引入完整 multi-agent。
- 不引入 PlannerAgent / MotorAgent / VisionAgent。
- 不做 Web UI。
- 不做复杂事件系统。
- 不做任务恢复。
- 不做长期记忆。
- 不把所有工具一次性改造成真正可取消的异步任务。

Phase 6 只引入最小可用的语言介入机制，为后续异步架构打基础。

## 3. 当前问题

当前 CLI 交互是同步阻塞结构：

```text
input("you> ")
  -> agent_loop()
  -> dispatch_tool_call()
  -> tool handler
  -> 工具执行完成后才回到 input("you> ")
```

这意味着：

- 工具执行时，主线程正在跑 agent loop 或工具函数。
- 用户无法在同一个终端里输入新的介入语句。
- 即使用户想说“停止”“悬停”“别继续了”，当前程序也要等工具返回。

对仿真来说这是体验问题；对真机来说这是安全风险。

## 4. 设计原则

Phase 6 应保持简单、直接、可读。

核心原则：

- 用户输入与工具执行解耦。
- 用户输入统一进入一个最小 `MessageBus`。
- 工具执行期间持续检查是否有介入消息。
- 任意工具执行期间收到新用户消息，都应中断当前工具。
- 飞行控制工具被中断时，必须先进入 hover。
- 中断后把控制权交回 agent，让 LLM 基于介入消息重新判断。
- 不把“是否中断”交给 LLM 自己理解，介入判断必须是代码硬逻辑。

## 5. 建议文件结构

新增文件：

```text
drone_agent/
  bus/
    __init__.py
    queue.py
    message_bus.py
    intervention.py
```

相关改动文件：

```text
drone_agent/
  runtime/
    runtime.py
    agent_loop.py
    tool_dispatcher.py
    task_state.py

  tools/
    registry.py
    flight.py
```

## 6. Bus 目录设计

Phase 6 的消息总线不放在 `runtime/` 中，而是放在内层 Python 包的独立目录：

```text
drone_agent/drone_agent/bus/
  __init__.py
  queue.py
  message_bus.py
  intervention.py
```

这样做的原因：

- `bus/` 后续会被 runtime、tools、未来 multi-agent 共同使用，不应该归属于某一个 runtime 文件。
- `queue.py` 只负责底层队列发布和消费。
- `message_bus.py` 负责把底层队列包装成当前 drone_agent 的消息语义。
- `intervention.py` 负责判断是否发生用户介入，并构造中断结果。

建议职责：

`queue.py`

- 提供最小同步消息队列。
- 负责 `publish()`、`consume()`、`try_consume()`、`has_pending()`。
- 第一版可以基于标准库 `queue.Queue`。

`message_bus.py`

- 定义 `UserMessage`。
- 定义 `MessageBus`。
- 提供 `publish_user_message()`、`get_next_user_message()`、`has_pending_user_message()`。

`intervention.py`

- 提供 `should_interrupt(context)`。
- 提供 `consume_intervention(context)`。
- 提供 `build_interrupted_result(...)`。
- 飞行工具被介入时，统一触发 hover 保护动作。

## 7. MessageBus 设计

第一版 `MessageBus` 只需要支持用户消息，不需要定义复杂事件类型。

建议数据结构：

```python
@dataclass(frozen=True)
class UserMessage:
    content: str
    created_at: float
```

建议接口：

```python
class MessageBus:
    def publish_user_message(self, content: str) -> None:
        ...

    def get_next_user_message(self) -> UserMessage | None:
        ...

    def has_pending_user_message(self) -> bool:
        ...
```

实现可以先基于标准库 `queue.Queue`，不需要一开始使用 `asyncio.Queue`。

原因：

- 当前 runtime 仍是同步架构。
- `queue.Queue` 可以安全地在输入线程和工具执行线程之间传递消息。
- 后续进入 multi-agent 或 asyncio 架构时，再替换为 async bus。

## 8. 同步队列能解决什么，不能解决什么

同步队列方案不是让 LLM 在一次请求还没返回时继续处理下一件事。

它真正解决的是：

```text
工具正在执行
  -> 输入线程仍然可以读取用户输入
  -> 用户输入进入 MessageBus
  -> 工具循环检查 MessageBus
  -> 发现介入后触发 EndCurrentTurn
```

也就是说，第一版语言介入依赖“输入线程 + 工具主动检查”的协作式中断。

它不能解决：

- LLM API 请求还没返回时，立即让同一个 LLM 同时处理介入消息。
- 阻塞式第三方 API 调用中途被强行取消。
- 不带检查点的长时间同步函数被立刻打断。

如果用户在 LLM 正在生成 tool call 时输入介入消息，第一版行为是：

1. 输入线程先把消息写入 `MessageBus`。
2. 当前 LLM 请求继续等待返回。
3. LLM 返回后，`tool_dispatcher.py` 在真正执行工具前先检查 bus。
4. 如果发现介入消息，直接抛出 `EndCurrentTurn`，不执行这次工具。
5. runtime 把介入消息作为新的 user message 交给 LLM。

因此，同步队列可以做到“用户输入不丢、工具执行前后可中断”，但不能做到“LLM 请求本身被即时取消并立即重入”。

## 9. 是否一次性切换为 asyncio.Queue

可以一次性切换为 `asyncio.Queue`，但会明显增加当前项目复杂度。

影响点包括：

- `runtime.py` 需要变成 async 主循环。
- 当前 `input()` 需要改为 async 输入或放进线程执行。
- OpenAI 兼容 LLM/VLM 调用要么换 async client，要么用 `asyncio.to_thread()` 包起来。
- `agent_loop.py`、`tool_dispatcher.py` 需要改成 async 调用链。
- 部分 tools 要改成 async，或者继续用 `to_thread()` 包同步函数。
- ROS2 `rclpy` executor 与 asyncio event loop 需要明确协作方式。

这会影响整体架构，但不是不可做。

客观判断：

- 如果 Phase 6 只解决“工具执行期间用户可以介入”，同步队列 + 输入线程更稳，改动更小。
- 如果下一步马上要进入 multi-agent，并且 Planner / Vision / Motor 都要并行运行，那可以考虑直接切 async。
- 当前项目还处在单体 agent 阶段，建议先实现同步 MessageBus，把介入语义跑通，再在 multi-agent 阶段整体迁移 async。

推荐策略：

```text
Phase 6：queue.Queue + 输入线程 + 协作式中断
Phase 9：asyncio.Queue + async runtime + multi-agent
```

这样不会在当前阶段同时改输入、LLM、tools、ROS2 executor 和消息通信，风险更可控。

## 10. 独立输入终端设计

当前实现不再默认依赖主终端中的 `input("you> ")`，而是拆成两个终端：

```text
输出终端
  -> 显示 agent / tool / state / ROS2 日志

输入终端
  -> 显示 you>
  -> 读取自然语言或 Y/N
  -> 通过本地 socket 发送给主进程
```

主进程内新增一个最小本地输入服务：

```text
runtime.py
  -> 创建 MessageBus
  -> 创建 InputServer
  -> 启动独立输入终端 input_terminal.py
  -> 从 MessageBus 消费用户消息
```

这样做的原因：

- 语言介入已经实现后，用户输入和运行时日志混在一个终端里，体验很差。
- 真机测试时，用户需要在不中断日志观察的前提下持续输入介入消息或 HITL 确认。
- 把输入端拆出去后，主终端可以只保留控制与观测输出。

第一版独立输入终端仍只支持：

- `exit`
- `quit`
- 普通自然语言输入
- HITL 的 `Y/N`

如果当前环境无法自动打开新终端，程序会回退到旧的单终端输入线程模式。

### 10.1 新增文件

```text
drone_agent/drone_agent/bus/input_server.py
drone_agent/drone_agent/bus/input_terminal.py
drone_agent/drone_agent/runtime/terminal.py
```

职责：

- `input_server.py`：主进程内的本地输入服务，把独立终端消息写入 `MessageBus`
- `input_terminal.py`：运行在新终端里的输入客户端
- `runtime/terminal.py`：负责拼装 `python -m drone_agent.bus.input_terminal ...` 命令并自动打开新终端

### 10.2 当前实现边界

当前 `MessageBus` 可以排队保存多条用户输入，但一次介入中断只消费一条消息。

这意味着：

- 用户可以连续输入多条消息，消息不会丢
- 当前工具被打断时，只会先取出队列中的第一条作为本次介入内容
- 剩余消息继续留在队列里，等待后续运行时循环消费

所以第一版支持“多条消息入队”，但还不支持“同一轮一次性合并处理多条介入消息”。

## 11. TaskState 扩展

当前 `TaskState` 已经有：

- `intervention_pending`
- `intervention_message`
- `active_tool_name`
- `active_tool_is_flight_tool`
- `current_phase`

Phase 6 建议增加最小方法：

```python
def mark_intervention(self, message: str) -> None:
    ...

def clear_intervention(self) -> None:
    ...
```

语义：

- `mark_intervention()`：记录用户介入消息，设置 `intervention_pending=True`，阶段改为 `interrupted`。
- `clear_intervention()`：LLM 已经处理介入消息后清空介入状态。

不建议第一版增加更多字段。现在的字段足够承载“有介入、介入内容是什么、当前工具是什么、是否飞行工具”。

## 12. ToolContext 扩展

当前 `ToolContext` 包含：

```text
controller
profile
session_id
task_state
```

Phase 6 建议扩展为：

```text
controller
profile
session_id
task_state
message_bus
```

这样 `tool_dispatcher.py` 和工具函数都可以访问同一个输入队列。

## 13. 介入检测位置

介入检测不是让 LLM 自己判断，也不是只在某个 prompt 中提示。

硬规则是：

```text
只要任务执行期间 MessageBus 中出现新的用户消息
  -> 取出这条消息
  -> 标记 TaskState.intervention_pending=true
  -> 如果当前执行的是飞行工具，先切 hover
  -> 抛出 EndCurrentTurn
  -> 当前轮结束
  -> 旧任务链路不继续
  -> runtime 把介入消息补充为新的 user message
```

第一版应把介入检测放在两个层级，是为了覆盖不同时间点。

### 13.1 dispatcher 层

位置：`runtime/tool_dispatcher.py`

在工具调用前检查：

- 如果 bus 已经有待处理用户消息，直接不执行工具。
- 标记 `TaskState.intervention_pending=True`。
- 抛出 `EndCurrentTurn`，把控制权交回 runtime。

这能处理“LLM 还在生成时用户已经输入，LLM 返回 tool call 后准备执行工具”的情况。

### 13.2 工具执行层

位置：`tools/flight.py` 等长时间工具内部循环。

在长时间循环中检查：

```python
if should_interrupt(context):
    ...
```

需要重点接入这些工具：

- `takeoff`
- `land`
- `rotate`
- `move`
- `timer`

这些工具都有可能持续数秒甚至更久。

短工具第一版可以只依赖 dispatcher 层检查：

- `current_position_status`
- `battery_status`
- `flight_mode_status`
- `take_photo`
- `analyze_view`
- `hover`
- `return_home`
- `disarm`

后续如果 `analyze_view` 的 VLM 调用耗时明显，再补充 VLM 调用级别的超时和中断。

## 14. 飞行工具中断规则

如果当前工具属于 `FLIGHT_TOOL_NAMES`，且检测到用户介入：

1. 从 `MessageBus` 取出介入消息。
2. `TaskState.mark_intervention(message)`。
3. 调用 hover 保护动作。
4. 当前工具返回：

```python
{
    "success": False,
    "error": "INTERRUPTED_BY_USER",
    "message": "tool interrupted by user input",
    "intervention_message": "...",
    "final_position_ned": ...
}
```

5. `tool_dispatcher.py` 识别 `INTERRUPTED_BY_USER`，抛出 `EndCurrentTurn`。
6. `agent_loop.py` 结束当前轮，不继续旧 tool call 链路。
7. 如果同一条 assistant message 中还有未执行的 `tool_calls`，`agent_loop.py` 会为剩余 `tool_call_id` 补 `SKIPPED_DUE_TO_TURN_END` 结果，避免下一轮请求违反 OpenAI tool call 消息协议。
8. `runtime.py` 把介入消息作为新的 user message 交给 LLM。

注意：飞行工具中断时的 hover 必须是代码动作，不应该让 LLM 决定是否 hover。

## 15. 非飞行工具中断规则

如果当前工具不是飞行控制工具，且检测到用户介入：

1. 从 `MessageBus` 取出介入消息。
2. `TaskState.mark_intervention(message)`。
3. 不执行 hover。
4. 当前工具返回 `INTERRUPTED_BY_USER`。
5. 当前轮结束。
6. 介入消息进入下一轮 LLM。

第一版重点是中断当前任务链路，而不是强行杀掉正在运行的第三方 API 调用。

## 16. 日志设计

现有会话日志保持不变：

```text
agent_messages.jsonl
tool_calls.jsonl
task_state.jsonl
```

Phase 6 建议增加介入相关字段，但不新增独立日志文件。

`task_state.jsonl` 中应记录：

- `intervention_pending`
- `intervention_message`
- `active_tool_name`
- `active_tool_is_flight_tool`
- `last_error: INTERRUPTED_BY_USER`

`tool_calls.jsonl` 中断工具结果应包含：

- `error: INTERRUPTED_BY_USER`
- `intervention_message`
- `final_position_ned`（飞行工具尽量返回）

`agent_messages.jsonl` 应记录介入消息本身，角色仍然是 `user`。

## 17. 终端输出建议

当前已有：

- `state>` 绿色
- `tool>`
- `agent>`
- `you>`

Phase 6 建议增加：

```text
intervention> 收到用户介入：悬停，不要继续
```

状态行可显示：

```text
state> interrupted move flight_tool=true error=INTERRUPTED_BY_USER
```

## 18. 与 agent_loop 的关系

Phase 6 不建议让 LLM 在同一轮继续处理介入消息。

推荐行为：

```text
旧用户任务
  -> LLM 调用工具 A
  -> 用户介入
  -> 工具 A 返回 INTERRUPTED_BY_USER
  -> agent_loop 补齐当前 assistant message 中剩余未执行 tool_call_id
  -> 当前轮 EndCurrentTurn
  -> runtime 把介入消息追加为新的 user message
  -> agent_loop 重新开始一轮
```

原因：

- 避免旧 tool call 链路继续执行。
- 让介入消息成为新的最高优先级用户输入。
- 与现有 `EndCurrentTurn` 机制一致。
- 保持 OpenAI tool call 消息序列合法，避免出现 assistant `tool_calls` 后缺少对应 tool message 的 400 错误。

## 19. 风险与限制

### 19.1 无法真正打断阻塞 API

如果某个工具正在执行不可中断的阻塞调用，例如 VLM API 请求，第一版不能立即终止底层请求。

处理方式：

- dispatcher 层可以阻止下一次工具调用。
- 工具返回后立即识别介入并结束当前轮。
- 后续再为 VLM 调用增加超时和更细粒度取消。

### 19.2 Python 线程无法安全强杀

不建议通过强杀线程来停止工具。

处理方式：

- 工具内部循环主动检查介入。
- 对飞行工具优先补检查点。
- 对外部 API 调用依赖超时参数。

### 19.3 hover 不是所有状态都一定成功

飞行工具中断时调用 hover 可能失败，例如位置无效、PX4 状态异常。

处理方式：

- hover 尝试失败也必须返回结构化结果。
- `TaskState` 仍然进入 `interrupted`。
- 后续 Phase 7 安全门补充更强的异常处理策略。

## 20. 实施顺序建议

建议按下面顺序实现：

1. 新增 `drone_agent/bus/__init__.py`。
2. 新增 `drone_agent/bus/queue.py`，实现底层同步队列发布和消费。
3. 新增 `drone_agent/bus/message_bus.py`，实现用户消息语义封装。
4. 新增 `drone_agent/bus/intervention.py`，集中放介入检测和结果构造。
5. 扩展 `TaskState`，增加 `mark_intervention()` 和 `clear_intervention()`。
6. 扩展 `ToolContext`，加入 `message_bus`。
7. 改造 `runtime.py`，把同步输入改为输入线程 + bus。
8. 在 `tool_dispatcher.py` 工具执行前检查介入。
9. 在 `timer()` 中接入介入检查，先验证非飞行工具中断。
10. 在 `takeoff()`、`move()`、`rotate()`、`land()` 的循环中接入介入检查。
11. 飞行工具介入时调用 hover 保护动作。
12. 更新日志字段，确保 `task_state.jsonl` 和 `tool_calls.jsonl` 能看出中断原因。
13. 写仿真验收用例。
14. 仿真验证后再考虑真机低风险场景。

## 21. 验收标准

### 21.1 非飞行工具介入

场景：

```text
用户输入：等待30秒
工具执行期间输入：停止等待
```

预期：

- `timer` 被中断。
- 工具返回 `INTERRUPTED_BY_USER`。
- 当前轮结束。
- 介入消息作为新的用户输入进入下一轮。
- 不触发 hover。

### 21.2 飞行工具介入

场景：

```text
用户输入：向前飞10米
工具执行期间输入：悬停，不要继续
```

预期：

- `move` 检测到介入。
- 立即尝试 hover。
- 工具返回 `INTERRUPTED_BY_USER`。
- `state>` 显示 `interrupted move flight_tool=true`。
- 当前轮结束。
- 不继续旧任务中的后续工具。
- 介入消息作为新的用户输入进入下一轮。

### 21.3 旧任务链路不继续

场景：

```text
用户输入：起飞3米，然后向前飞5米，然后降落
在向前飞期间输入：停止，保持悬停
```

预期：

- `move` 中断。
- 不继续执行后续 `land`。
- LLM 下一轮优先处理“停止，保持悬停”。

### 21.4 日志可追踪

预期：

- `task_state.jsonl` 能看到 `intervention_pending=true`。
- `tool_calls.jsonl` 能看到 `error=INTERRUPTED_BY_USER`。
- `agent_messages.jsonl` 能看到介入消息。

## 22. 与后续阶段的关系

Phase 6 完成后，项目将具备最小可用的“可打断执行”能力。

它为后续阶段提供基础：

- Phase 7 安全门：可以在安全门触发时复用同一套中断和 hover 机制。
- Skills：skill 执行过程可以遵守统一的介入规则。
- Multi-Agent：后续 PlannerAgent、MotorAgent、VisionAgent 可以复用 `MessageBus` 和 `TaskState`，而不是重新设计通信机制。

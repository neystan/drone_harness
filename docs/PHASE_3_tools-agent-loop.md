# Drone Agent Phase 3：Tools 与 Agent Loop 实施计划

> **给执行者的要求：** 该计划只用于 review 和后续逐步执行。用户确认前，不要开始改代码。

**目标：** 将当前 `/download/takeoff.py` 中已经验证过的 Function Calling 工具和 Agent Loop 迁移到 `/download/drone_agent` 包结构中，不新增飞行能力。

**架构方向：** `Px4Controller` 继续作为直接的 ROS2/PX4 DDS 控制对象，不引入为了测试服务的复杂依赖注入。LLM 可调用函数放在 `drone_agent/tools`，模型客户端和 prompt 放在 `drone_agent/llm`，工具调度和对话循环放在 `drone_agent/core`。

**技术栈：** Python 3.10+、ROS2 `rclpy`、`px4_msgs`、OpenAI-compatible Chat Completions。

---

## 一、文件范围

本阶段计划新增：

- `drone_agent/drone_agent/tools/__init__.py`
- `drone_agent/drone_agent/tools/status.py`
- `drone_agent/drone_agent/tools/flight.py`
- `drone_agent/drone_agent/tools/perception.py`
- `drone_agent/drone_agent/tools/schemas.py`
- `drone_agent/drone_agent/tools/registry.py`
- `drone_agent/drone_agent/llm/__init__.py`
- `drone_agent/drone_agent/llm/client.py`
- `drone_agent/drone_agent/llm/prompts.py`
- `drone_agent/drone_agent/core/tool_dispatcher.py`
- `drone_agent/drone_agent/core/agent_loop.py`

本阶段计划修改：

- `drone_agent/drone_agent/core/runtime.py`
- `drone_agent/pyproject.toml`

本阶段计划增加测试：

- `drone_agent/tests/unit/test_tools_schemas.py`
- `drone_agent/tests/unit/test_tools_registry.py`
- `drone_agent/tests/unit/test_tool_dispatcher.py`
- `drone_agent/tests/unit/test_llm_prompts.py`

---

## 二、模块职责

### `tools/schemas.py`

集中存放 OpenAI Function Calling 的 tool schema，名称与当前 `takeoff.py` 保持一致：

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

该文件只定义 schema，不执行任何飞控动作。

### `tools/registry.py`

定义工具注册表，负责把 tool name 映射到 Python 函数。

建议数据结构：

```python
@dataclass(frozen=True)
class ToolContext:
    controller: Any
    profile: RuntimeProfile


@dataclass(frozen=True)
class ToolDefinition:
    name: str
    schema: dict[str, Any]
    handler: Callable[[ToolContext, dict[str, Any]], dict[str, Any]]
```

说明：

- `ToolContext` 是为了让工具拿到 `controller` 和 `profile`。
- 它不是为了测试而设计的抽象，而是 tool 调用天然需要的运行上下文。
- 后续如果加入高级 `skills/`，也可以复用这层 registry。

### `tools/status.py`

迁移当前 `takeoff.py` 的状态查询工具：

- `current_position_status(controller)`
- `battery_status(controller)`
- `flight_mode_status(controller)`

其中 enum 名称解析继续使用 `drone_agent/px4/status.py` 中已有 helper。

### `tools/flight.py`

迁移当前 `takeoff.py` 的飞行动作工具：

- `takeoff(controller, height, profile)`
- `land(controller, profile)`
- `disarm(controller)`
- `timer(seconds)`
- `hover(controller)`
- `return_home(controller)`
- `rotate(controller, direction, degrees, profile)`
- `move(controller, x, y, z, profile)`

关键要求：

- 保留当前 `takeoff.py` 的控制语义。
- 不新增新飞行动作。
- 不改变 body FRD 到 NED 的运动语义。
- 安全阈值优先使用 profile 中已有字段，例如最大起飞高度、最大相对移动距离、最大垂直移动距离、最大旋转角度、动作超时时间。
- 保留 timeout 后自动 hover 的契约。
- 保留 `requires_user_confirmation`、`target_position_ned`、`final_position_ned` 等返回字段。

### `tools/perception.py`

本阶段不完整迁移视觉能力，只提供明确占位：

- `take_photo(context, arguments)` 返回 `PERCEPTION_NOT_MIGRATED`
- `analyze_view(context, arguments)` 返回 `PERCEPTION_NOT_MIGRATED`

原因：

- `take_photo` 和 `analyze_view` 依赖图片保存、VLM 调用、图像编码、JSON 解析等视觉层能力。
- 这些内容更适合在 Phase 4 单独迁移到 `vision/`。
- 但 schema 和 registry 先保留这两个工具名，避免 prompt 和 tool list 结构反复变化。

### `llm/client.py`

负责创建 OpenAI-compatible client：

```python
def create_llm_client(profile: RuntimeProfile) -> OpenAI:
    return OpenAI(api_key=profile.llm.api_key, base_url=profile.llm.base_url)
```

真实 API key 继续来自环境变量，经 `config/loader.py` 注入到 `RuntimeProfile`。

### `llm/prompts.py`

存放中文 system prompt，从 `takeoff.py` 迁移。

必须保留的安全语义：

- 如果工具返回 `requires_user_confirmation=true`，agent 必须停止后续飞行动作。
- `target_position_ned` 只是目标位置。
- `final_position_ned` 才是工具结束时的实际位置。
- 用户只是聊天或提问时，直接中文回答。

### `core/tool_dispatcher.py`

负责解析模型返回的 tool call，并调用 registry 中对应 handler。

职责：

- 打印可见工具调用日志，例如 `tool> calling move args={...}`。
- 解析 JSON 参数。
- JSON 参数非法时返回 `INVALID_TOOL_ARGUMENTS`。
- tool name 不存在时返回 `UNSUPPORTED_TOOL`。
- 不直接写飞行动作逻辑。

### `core/agent_loop.py`

负责 OpenAI-compatible Function Calling 循环。

建议函数：

```python
def run_agent_turn(client, model: str, messages: list[dict], context: ToolContext) -> str:
    ...


def run_interactive_agent(client, model: str, context: ToolContext) -> None:
    ...
```

关键要求：

- 每轮最多 50 次 tool calling，保持当前 `takeoff.py` 的上限。
- tool result 写回 messages。
- 如果某个工具结果包含 `requires_user_confirmation=true`，本轮停止继续工具调用，并向用户输出异常信息。
- 保留终端交互形式：`you>` 输入，`agent>` 输出。

### `core/runtime.py`

本阶段从 stub 过渡到可启动真实 runtime，但保留轻量检查路径。

建议接口：

```python
def start_runtime(
    profile_name: str,
    task: str | None = None,
    interactive: bool = True,
    start_ros: bool = False,
) -> RuntimeStartResult:
    ...
```

语义：

- `start_ros=False`：保持 Phase 1 的 no-ROS profile/CLI 检查能力。
- `start_ros=True`：初始化 `rclpy`、`SingleThreadedExecutor`、`Px4Controller`、LLM client，并进入 agent loop。
- `task is not None`：执行一次自然语言任务后退出。
- `interactive=True`：进入交互式 CLI loop。

### `pyproject.toml`

补充运行依赖：

- `openai>=1.0`

ROS2、`px4_msgs`、`cv_bridge`、`sensor_msgs` 不建议写进 pip 依赖，因为它们来自 ROS2 环境。

---

## 三、执行任务

### Task 1：建立 Tools Schema 与 Registry

- [ ] 新增 `tools/schemas.py`，迁移所有 tool schema。
- [ ] 新增 `tools/registry.py`，建立 name 到 handler 的映射。
- [ ] 新增 `tools/perception.py`，保留 `take_photo` 和 `analyze_view` 的明确占位返回。
- [ ] 新增测试，确认 tool name 顺序和当前 `takeoff.py` 一致。

验收：

- `get_tool_schemas()` 返回 13 个工具。
- 工具名包含当前 `takeoff.py` 的全部 Function Calling 工具。
- perception 工具虽然注册，但返回明确的 `PERCEPTION_NOT_MIGRATED`。

### Task 2：迁移 Status 与 Flight Tools

- [ ] 新增 `tools/status.py`。
- [ ] 新增 `tools/flight.py`。
- [ ] 将安全阈值从硬编码逐步改为读取 `profile.safety`。
- [ ] 保持工具返回结构与当前 `takeoff.py` 兼容。

验收：

- 状态工具仍返回 `success` 和可读字段。
- 飞行动作工具保留原有错误码风格。
- timeout 返回必须包含 `requires_user_confirmation=true`。
- 涉及目标位置的动作必须保留 `target_position_ned` 和 `final_position_ned`。

### Task 3：迁移 LLM Prompt、Client、Dispatcher、Agent Loop

- [ ] 新增 `llm/prompts.py`。
- [ ] 新增 `llm/client.py`。
- [ ] 新增 `core/tool_dispatcher.py`。
- [ ] 新增 `core/agent_loop.py`。
- [ ] 移除长串 `if call.function.name == ...` 结构，改用 registry。

验收：

- dispatcher 能处理非法 JSON 参数。
- dispatcher 能处理未知工具名。
- prompt 中保留确认中断、安全字段解释、中文回复要求。
- agent loop 遇到 `requires_user_confirmation=true` 后停止继续工具调用。

### Task 4：Runtime 接线

- [ ] 修改 `core/runtime.py`。
- [ ] 保留 `start_ros=False` 的 profile 检查路径。
- [ ] 增加 `start_ros=True` 的真实 ROS2 runtime 路径。
- [ ] CLI 默认暂时可以继续走 no-ROS smoke 路径，真实运行入口后续可通过参数或下一阶段打开。

验收：

- 不启动 ROS2 时，当前 smoke 命令仍能通过。
- 启动真实 runtime 的代码路径清晰，创建对象顺序为：profile -> rclpy -> controller -> executor thread -> llm client -> agent loop。
- 退出时 shutdown executor、destroy node、`rclpy.shutdown()`。

### Task 5：验证

- [ ] 在 `/download/drone_agent` 运行 `pytest -v`。
- [ ] 对不依赖 ROS2 import 的新模块运行 `python3 -m py_compile`。
- [ ] 用假环境变量运行 no-ROS CLI smoke：

```bash
DRONE_AGENT_LLM_API_KEY=x DRONE_AGENT_VLM_API_KEY=y drone_agent --profile sim --task "查询状态"
```

- [ ] 扫描是否误提交真实 key：

```bash
grep -R "sk-" -n drone_agent tests pyproject.toml README.md
```

---

## 四、暂不做的内容

本阶段不做：

- 不完整迁移 VLM。
- 不新增搜索目标、靠近目标等高级能力。
- 不引入 `skills/` 高级任务层。
- 不引入 `SimAdapter` / `RealAdapter`。
- 不为了测试改造 `Px4Controller` 的直接 ROS2 结构。

---

## 五、Review 重点

请重点确认：

- `tools/` 这个命名是否继续符合你的预期。
- `perception.py` 本阶段只做占位是否可以接受。
- `start_runtime(..., start_ros=False)` 继续保留 no-ROS smoke 路径是否合适。
- CLI 是否要在 Phase 3 就增加 `--start-ros`，还是 Phase 4 再打开真实运行入口。

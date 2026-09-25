# Drone Agent Phase 4：Vision、安全、日志与运行验收实施计划

> **给执行者的要求：** 该计划只用于 review 和后续逐步执行。用户确认前，不要开始改代码。

**目标：** 在 Phase 1-3 已完成的包结构、PX4 控制层、Tools 与 Agent Loop 基础上，把当前占位的视觉工具迁移完整，打开真实 ROS2 运行入口，补齐最小安全确认与日志能力，并形成仿真/真机验收流程。

**架构方向：** 继续保持 `Px4Controller` 直接负责 ROS2/PX4 DDS 通信。视觉能力进入 `drone_agent/vision`，LLM 可调用工具仍只暴露在 `drone_agent/tools`。安全和日志以轻量、直接、可读为原则，不引入复杂任务编排器，也不新增高级自主飞行能力。

**技术栈：** Python 3.10+、ROS2 `rclpy`、`px4_msgs`、`sensor_msgs`、`cv_bridge`、OpenCV、OpenAI-compatible Chat Completions、OpenAI-compatible VLM。

---

## 一、Phase 4 前置状态

当前已经具备：

- `config/`：`sim.yaml`、`real.yaml`、profile schema 和 loader。
- `px4/`：直接的 `Px4Controller(Node)`、PX4 topic、坐标转换、状态 helper。
- `tools/`：飞行工具、状态工具、tool schemas、registry。
- `llm/`：LLM client 和中文 system prompt。
- `core/`：tool dispatcher、agent loop、`start_runtime(..., start_ros=False/True)`。

当前还缺：

- `take_photo` 和 `analyze_view` 仍是 `PERCEPTION_NOT_MIGRATED` 占位。
- CLI 还没有显式参数启动真实 ROS2 runtime。
- 安全确认规则还没有集中收口到 `core/safety.py`。
- 工具调用和任务过程没有落地日志。
- 缺少明确的仿真/真机手动验收步骤。

---

## 二、文件范围

本阶段计划新增：

- `drone_agent/drone_agent/vision/__init__.py`
- `drone_agent/drone_agent/vision/image_store.py`
- `drone_agent/drone_agent/vision/vlm.py`
- `drone_agent/drone_agent/core/safety.py`
- `drone_agent/drone_agent/logging/__init__.py`
- `drone_agent/drone_agent/logging/task_log.py`
- `drone_agent/tests/unit/test_vision_image_store.py`
- `drone_agent/tests/unit/test_vision_vlm.py`
- `drone_agent/tests/unit/test_safety.py`
- `drone_agent/tests/unit/test_task_log.py`
- `drone_agent/tests/unit/test_cli_runtime_mode.py`

本阶段计划修改：

- `drone_agent/drone_agent/tools/perception.py`
- `drone_agent/drone_agent/tools/registry.py`
- `drone_agent/drone_agent/core/agent_loop.py`
- `drone_agent/drone_agent/core/tool_dispatcher.py`
- `drone_agent/drone_agent/core/runtime.py`
- `drone_agent/drone_agent/cli.py`
- `drone_agent/README.md`
- `drone_agent/pyproject.toml`

---

## 三、模块职责

### `vision/image_store.py`

负责图片保存，不调用模型，不读取 ROS topic。

建议函数：

```python
def save_photo(frame, photo_save_dir: str) -> dict:
    ...


def save_analysis_frame(frame, analysis_save_dir: str) -> Path:
    ...
```

要求：

- 使用 profile 中的 `storage.photo_save_dir` 和 `storage.analysis_save_dir`。
- 目录不存在时自动创建。
- 保存失败时返回或抛出明确错误。
- 返回图片路径、宽度、高度。

### `vision/vlm.py`

负责视觉模型调用和结果归一化。

从 `/download/takeoff.py` 迁移以下逻辑：

- `encode_image_to_data_url`
- `build_analyze_view_prompt`
- `extract_json_object`
- `normalize_offset`
- `normalize_confidence`
- `derive_suggested_action`
- `normalize_vlm_result`
- `call_vlm`

要求：

- VLM client 使用 `profile.vlm.base_url`、`profile.vlm.model`、`profile.vlm.api_key`。
- 不硬编码 key、base URL、model。
- `target_description` 为空时只做场景描述。
- `target_description` 存在时返回目标识别结果和建议动作。
- 保留当前动作建议集合：`rotate_right_search`、`rotate_left_search`、`move_right`、`move_left`、`move_forward`、`take_photo`、`hold_position`。
- 当前未发现目标时，`derive_suggested_action()` 默认返回 `rotate_left_search`，与 `visual-search` 的持续向左粗搜索策略保持一致。
- 当前已经发现目标时，`suggested_action` 根据 `target_position` 和 `center_offset_x` 推导：目标偏左建议左转，目标偏右建议右转，目标居中且偏移足够小时建议拍照。

### `tools/perception.py`

替换 Phase 3 占位实现。

工具语义：

- `take_photo(context, arguments)` 从 `context.controller.latest_rgb_frame` 取最新图像，保存到 profile 配置的图片目录。
- `analyze_view(context, arguments)` 保存分析帧，调用 VLM，返回归一化结果。

错误语义：

- 无相机帧：`IMAGE_NOT_READY`
- 图片目录创建失败：`PHOTO_DIR_CREATE_FAILED`
- 图片保存失败：`PHOTO_SAVE_FAILED`
- 分析帧保存失败：`ANALYSIS_FRAME_SAVE_FAILED`
- VLM 未启用：`VLM_DISABLED`
- VLM 调用失败：`VLM_ANALYSIS_FAILED`

### `core/safety.py`

安全策略做轻量集中化，不重写飞行工具。

建议函数：

```python
def should_stop_after_tool_result(profile: RuntimeProfile, result: dict) -> bool:
    ...


def requires_real_flight_confirmation(profile: RuntimeProfile, tool_name: str) -> bool:
    ...
```

职责：

- 判断 `requires_user_confirmation=true` 后是否停止 agent loop。
- 根据 `profile.safety.stop_after_requires_confirmation` 决定是否阻断继续工具调用。
- 为真机默认确认策略预留入口。

本阶段不强制把所有参数校验搬出 `tools/flight.py`，避免过度拆分。已有飞行工具仍保留直接、可读的校验逻辑。

### `logging/task_log.py`

提供 JSONL 任务日志和工具调用日志。

建议数据：

- 时间戳。
- profile name。
- user input。
- assistant message。
- tool name。
- tool arguments。
- tool result。
- `requires_user_confirmation`。

建议函数：

```python
def append_jsonl(log_dir: str, filename: str, event: dict) -> None:
    ...


def log_tool_call(profile: RuntimeProfile, tool_name: str, arguments: dict, result: dict) -> None:
    ...


def log_agent_message(profile: RuntimeProfile, role: str, content: str) -> None:
    ...
```

要求：

- 日志目录来自 `profile.storage.log_dir`。
- 日志写失败不能导致飞行工具失败，只打印 warning 或静默降级。
- 不记录 API key。

### `core/tool_dispatcher.py`

增加日志接线。

职责变化：

- 继续打印 `tool> calling ...`，保持终端可见。
- 调用工具后写 tool log。
- JSON 参数非法、未知工具也写入日志，便于排查。

### `core/agent_loop.py`

接入安全策略和任务日志。

职责变化：

- 用户输入写 task log。
- assistant 最终回复写 task log。
- 工具结果交给 `core/safety.py` 判断是否停止继续 tool calling。
- 保持当前 `requires_user_confirmation` 中断语义。

### `core/runtime.py`

完善真实 runtime 的生命周期。

要求：

- `start_ros=False` 保持 profile/CLI smoke 能力。
- `start_ros=True` 保持当前创建顺序：profile -> rclpy -> controller -> executor thread -> LLM client -> agent loop。
- 启动失败时尽量关闭 executor、destroy node、`rclpy.shutdown()`。
- 可选打印当前 profile、camera topic、log dir，便于运行时确认。

### `cli.py`

打开真实运行入口。

建议新增参数：

```bash
drone_agent --profile sim --start-ros
drone_agent --profile sim --start-ros --task "起飞到1米"
drone_agent_sim --start-ros
drone_agent_real --start-ros
```

说明：

- 默认不传 `--start-ros` 时，继续走 no-ROS smoke 路径。
- 这样普通开发环境仍可运行 CLI 和测试。
- 真正控制 PX4 时显式加 `--start-ros`，降低误启动风险。

---

## 四、执行任务

### Task 1：迁移图片保存层

- [ ] 新增 `vision/__init__.py`。
- [ ] 新增 `vision/image_store.py`。
- [ ] 从 `takeoff.py` 迁移图片保存和文件命名逻辑。
- [ ] 添加单元测试覆盖目录创建、保存失败返回、成功返回图片尺寸。

验收：

- 图片保存目录来自 profile storage。
- 不再硬编码 `/home/hw/picture`。
- `take_photo` 后续只调用 image store，不直接操作路径细节。

### Task 2：迁移 VLM 逻辑

- [ ] 新增 `vision/vlm.py`。
- [ ] 迁移 VLM prompt、图片编码、JSON 提取、结果归一化。
- [ ] VLM client 使用 profile 中的 `vlm` 配置。
- [ ] 添加单元测试覆盖 JSON 提取、offset/confidence clamp、建议动作推导。

验收：

- VLM 逻辑不依赖 ROS2。
- API key 不出现在源码。
- `target_description=None` 与有目标搜索两种模式都保留。

### Task 3：替换 Perception 占位工具

- [ ] 修改 `tools/perception.py`。
- [ ] 实现 `take_photo(context, arguments)`。
- [ ] 实现 `analyze_view(context, arguments)`。
- [ ] `profile.vlm.enabled=false` 时返回 `VLM_DISABLED`。

验收：

- `take_photo` 不再返回 `PERCEPTION_NOT_MIGRATED`。
- `analyze_view` 不再返回 `PERCEPTION_NOT_MIGRATED`。
- 无相机帧时返回 `IMAGE_NOT_READY`。
- 保存失败和 VLM 失败有明确错误码。

### Task 4：安全确认收口

- [ ] 新增 `core/safety.py`。
- [ ] 在 `agent_loop.py` 中使用 `should_stop_after_tool_result()`。
- [ ] 保留现有 `requires_user_confirmation=true` 中断行为。
- [ ] 添加测试覆盖 `stop_after_requires_confirmation=true/false`。

验收：

- 安全确认逻辑集中在 `core/safety.py`。
- 飞行工具不用为了这件事继续扩散 agent loop 逻辑。
- 默认 profile 行为与当前 Phase 3 一致。

### Task 5：最小任务日志

- [ ] 新增 `logging/__init__.py`。
- [ ] 新增 `logging/task_log.py`。
- [ ] 在 dispatcher 中记录 tool call 和 tool result。
- [ ] 在 agent loop 中记录用户输入和 assistant 输出。
- [ ] 日志写失败不影响飞行流程。

验收：

- 日志写入 profile 的 `storage.log_dir`。
- JSONL 每行是独立 JSON 对象。
- 日志不包含 `profile.llm.api_key` 或 `profile.vlm.api_key`。

### Task 6：CLI 打开真实 ROS2 入口

- [ ] 修改 `cli.py`，新增 `--start-ros`。
- [ ] 将 `--start-ros` 传给 `start_runtime(..., start_ros=True)`。
- [ ] 更新 CLI 单元测试。
- [ ] 更新 README 启动说明。

验收：

- 不传 `--start-ros` 时，当前 smoke 行为不变。
- 传 `--start-ros` 时进入真实 ROS2 runtime 路径。
- `drone_agent_sim --start-ros` 和 `drone_agent_real --start-ros` 都能解析参数。

### Task 7：仿真与真机验收文档

- [ ] 在 README 增加手动验收步骤。
- [ ] 写明仿真命令、真机命令、环境变量、ROS2/PX4 前置条件。
- [ ] 写明真机测试顺序：状态查询 -> 电池查询 -> 低高度起飞 -> hover -> land。

验收：

- 用户能按 README 明确区分 smoke、仿真、真机三种运行方式。
- 真机默认流程不会跳过状态检查。

---

## 五、验证命令

基础验证：

```bash
cd /download/drone_agent
pytest -v
```

不启动 ROS2 的 smoke：

```bash
DRONE_AGENT_LLM_API_KEY=x DRONE_AGENT_VLM_API_KEY=y \
drone_agent --profile sim --task "查询状态"
```

真实仿真入口语法检查：

```bash
DRONE_AGENT_LLM_API_KEY=x DRONE_AGENT_VLM_API_KEY=y \
drone_agent_sim --start-ros --task "查询状态"
```

说明：这条命令需要 ROS2、PX4 DDS、`px4_msgs`、相机 topic 等运行环境。普通 CI 或无 ROS 环境不要求通过完整运行。

secret 扫描：

```bash
grep -R "sk-" -n drone_agent tests pyproject.toml README.md
```

---

## 六、暂不做的内容

本阶段不做：

- 不新增高级搜索/靠近/巡检 skill。
- 不引入 Web UI。
- 不改 `Px4Controller` 为 adapter 或依赖注入结构。
- 不把 AirSim 相机 topic 发布逻辑纳入本项目。
- 不做复杂任务规划器。
- 不把所有飞行参数校验搬到 `core/safety.py`。

---

## 七、Review 重点

请重点确认：

- Phase 4 是否应该同时包含视觉迁移、日志、安全、CLI 真实入口。
- `--start-ros` 作为显式真实运行开关是否符合你的使用习惯。
- 视觉工具是否应该在 Phase 4 完整迁移，还是继续延后。
- 日志先做 JSONL 是否足够。
- 真机验收顺序是否需要更保守。

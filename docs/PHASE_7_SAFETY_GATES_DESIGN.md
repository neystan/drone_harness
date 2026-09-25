# Phase 7：起飞前安全门设计文档

日期：2026-06-18

## 1. 目标

Phase 7 的目标不是把所有安全检查统一抽到外层，而是直接在 `takeoff()` 里补少量真机前置检查。

这层安全门只解决一个问题：

- 在真机起飞前，先做一次最小的起飞前检查。
- 检查失败时，直接拒绝起飞。
- 不把是否继续交给 LLM 判断。

## 2. 当前边界

当前 `drone_agent` 已经有两类安全逻辑：

### 2.1 外层通用安全逻辑

当前已在 `runtime/safety.py` 和 `tool_dispatcher.py` 中实现：

- `human_in_the_loop_for_flight_tools`
- `INTERRUPTED_BY_USER`
- `*_TIMEOUT` 结束当前轮
- 飞行工具被语言介入时 hover

这些逻辑保留，不属于 Phase 7 新增内容。

### 2.2 工具内部状态检查

当前 `tools/flight.py` 中各工具本来就有自己的前置检查。

例如 `takeoff()` 已经检查：

- `height` 类型是否合法
- `height` 是否大于 0
- `height` 是否超过 `max_takeoff_height_m`
- 本地位置是否有效
- 当前是否已经在空中

`land()`、`hover()`、`return_home()` 也各自检查：

- 本地位置是否有效
- 当前是否已经在空中或地面

这些检查本来就应该继续留在各自工具函数里。

## 3. Phase 7 不做什么

Phase 7 不做：

- 不增加“所有飞行工具统一前置检查”。
- 不把 `takeoff`、`land`、`move`、`rotate` 的已有状态检查再复制到外层。
- 不增加移动后位置偏差检查。
- 不增加旋转后 yaw 偏差检查。
- 不增加 VLM 低置信度运动限制。
- 不重写 `tools/flight.py` 的基本参数校验。
- 不引入复杂安全策略引擎。

原因很直接：

- 工具函数里已经有检查，再加一层就是重复。
- 当前项目阶段更需要简单、直接、可读。

## 4. Phase 7 要做什么

Phase 7 只补“`takeoff()` 内的真机起飞前检查”。

触发时机：

- 仅在 `takeoff` 真正执行前。
- 仅在工具内部已有参数检查通过之后。
- 仅在真机或需要更保守 profile 的场景下启用。

它的定位不是替代 `takeoff()` 现有检查，而是在现有 `takeoff()` 逻辑里直接补几项真机前置条件。

## 5. 建议新增配置

建议只增加和“起飞前检查”直接相关的少量字段：

```yaml
safety:
  human_in_the_loop_for_flight_tools: true
  max_takeoff_height_m: 3
  max_relative_move_m: 5
  max_vertical_move_m: 2
  max_rotation_deg: 180
  action_timeout_s: 20
  hover_on_timeout: true

  pre_takeoff_gate_enabled: true
  require_battery_status_for_takeoff: true
  min_battery_percent_for_takeoff: 30
  require_local_position_valid_for_takeoff: true
  require_px4_status_ready_for_takeoff: true
```

字段说明：

- `pre_takeoff_gate_enabled`：是否启用起飞前安全门。
- `require_battery_status_for_takeoff`：起飞前是否要求电池状态可读。
- `min_battery_percent_for_takeoff`：真机起飞最低电量阈值。
- `require_local_position_valid_for_takeoff`：起飞前是否要求本地位置有效。
- `require_px4_status_ready_for_takeoff`：起飞前是否要求 PX4 状态可读。

## 6. 起飞前安全门内容

Phase 7 建议只做这几项检查：

### 6.1 PX4 状态可读

检查内容：

- `vehicle_status` 是否已经收到有效数据。

失败行为：

- 返回 `TAKEOFF_GATE_STATUS_UNAVAILABLE`
- 不执行起飞

### 6.2 本地位置有效

检查内容：

- 本地位置是否有效。

说明：

- `takeoff()` 当前已经有 `_wait_for_valid_position()` 检查。
- 如果你希望完全避免重复，就继续只保留在 `takeoff()` 内部。

所以这项不是新增重点。

### 6.3 电池状态可读

检查内容：

- 是否收到电池状态。

失败行为：

- 返回 `TAKEOFF_GATE_BATTERY_UNAVAILABLE`
- 不执行起飞

### 6.4 电量阈值

检查内容：

- 真机电量是否高于 `min_battery_percent_for_takeoff`

失败行为：

- 返回 `TAKEOFF_GATE_BATTERY_TOO_LOW`
- 不执行起飞

## 7. 推荐实现位置

推荐只修改：

```text
drone_agent/tools/flight.py
```

直接在 `takeoff()` 内增加少量真机前置检查：

- 电池状态是否可读
- 电量是否高于阈值
- PX4 状态是否可读

返回方式继续沿用当前工具返回格式，不新增额外中间层。

## 8. 推荐实现顺序

原因：

- Phase 7 只针对 `takeoff`
- 不需要把 dispatcher 改成“所有工具都先过一遍安全门”
- `takeoff` 自己最清楚什么时候该检查、什么时候该返回

建议顺序：

```text
takeoff(height)
  -> 检查参数类型
  -> 检查 height > 0
  -> 检查 height <= max_takeoff_height_m
  -> 检查 PX4 状态是否可读
  -> 检查电池状态是否可读
  -> 检查电量是否高于阈值
  -> _wait_for_valid_position(...)
  -> 检查是否已在空中
  -> 执行起飞
```

按你当前偏好，`_wait_for_valid_position()` 继续保留，不做抽象。

## 9. 错误码建议

Phase 7 建议只增加少量错误码：

```text
TAKEOFF_GATE_STATUS_UNAVAILABLE
TAKEOFF_GATE_BATTERY_UNAVAILABLE
TAKEOFF_GATE_BATTERY_TOO_LOW
```

继续沿用当前工具返回格式：

```python
{
    "success": False,
    "error": "TAKEOFF_GATE_BATTERY_TOO_LOW",
    "message": "battery is below takeoff safety threshold",
}
```

## 10. 建议修改文件

建议修改：

```text
drone_agent/config/schema.py
drone_agent/config/loader.py
drone_agent/config/profiles/sim.yaml
drone_agent/config/profiles/real.yaml
drone_agent/tools/flight.py
docs/PROJECT_ARCHITECTURE.md
docs/legacy_specs/DRONE_AGENT_SPEC.md
```

不建议修改：

```text
drone_agent/runtime/tool_dispatcher.py
drone_agent/runtime/safety.py
drone_agent/tools/perception.py
drone_agent/px4/status.py
```

因为这一阶段不需要把安全门扩散到其它工具。

## 11. 验收标准

仿真验收：

- `pre_takeoff_gate_enabled=false` 时，仿真起飞行为与当前版本一致。
- 现有高度/距离/旋转角/超时限制不受影响。

真机验收：

- 电池状态不可读时，拒绝起飞。
- 电量低于阈值时，拒绝起飞。
- 起飞 gate 失败后，本轮直接结束，不继续旧任务链路。
- 用户介入、超时、HITL 行为保持当前实现不变。

## 12. 最终结论

Phase 7 最合理的收缩版是：

- 不做一套大的“安全门系统”
- 不重复 `takeoff()`、`land()`、`move()`、`rotate()` 内部已有检查
- 直接在 `takeoff()` 内补真机前置检查
- 只新增少量 profile 字段
- 只改少量文件

这更符合当前项目的阶段和你的偏好。

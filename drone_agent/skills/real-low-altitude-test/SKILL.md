---
name: real-low-altitude-test
description: 当用户要求真机低高度试飞、起飞测试、悬停测试或低高度移动测试时使用，指导 agent 先检查电池、飞行模式和当前位置，再执行保守飞行动作。
enabled: false
mode: ["sim", "real"]
---

# Real Low Altitude Test

## 使用场景

用户要求真机低高度试飞、起飞测试、悬停测试或低高度移动测试时使用。

## 工作流程

1. 先调用 `battery_status` 查看电池状态。
2. 再调用 `flight_mode_status` 查看 PX4 飞行状态。
3. 再调用 `current_position_status` 查看当前位置是否可读。
4. 只执行低高度 `takeoff`。
5. 起飞后优先 `hover` 或 `timer`，不要立即复杂移动。
6. 如果需要移动，只执行小距离 `move`。
7. 测试结束后调用 `land`。

## 可调用工具

推荐工具顺序：

```text
battery_status -> flight_mode_status -> current_position_status -> takeoff -> hover/timer -> land
```

## 安全约束

- 真机测试优先低高度。
- 起飞前必须查看电池、飞行模式和当前位置。
- 不要连续执行多个移动动作。
- 异常时优先悬停或降落。
- 飞行动作仍然必须遵守 HITL、语言介入和起飞前安全门。

## 失败处理

如果状态查询失败，停止起飞并向用户说明失败原因。

如果飞行动作失败或被用户介入，结束当前任务链路，等待用户重新指令。

## 反例

仿真测试、视觉搜索、普通状态查询不使用这个 skill。

## 示例

用户：

```text
真机低高度起飞测试一下
```

推荐工具调用顺序：

```text
battery_status()
flight_mode_status()
current_position_status()
takeoff(height=1)
timer(seconds=5)
land()
```

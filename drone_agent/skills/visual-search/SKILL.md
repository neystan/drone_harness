---
name: visual-search
description: 当用户要求寻找目标、搜索目标、观察环境、判断目标是否出现在画面中，或让无人机对准某个目标时使用，指导 agent 分步完成视觉搜索和目标对准。
enabled: true
mode: ["sim", "real"]
---

# Visual Search

## 使用场景

用户要求寻找某个目标、判断目标是否出现、观察环境、或让无人机对准目标时使用。

## 工作流程

1. 先调用 `analyze_view` 判断当前画面是否能看到目标。
2. 如果没有看到目标，先通过 `rotate` 观察周围一圈的情况。
3. 当前前视相机的水平视野约为 94 度。
4. 未找到目标时，粗搜索优先使用 90 度的大步旋转。
5. 粗搜索时尽量持续向左旋转，避免来回切换方向。
6. 如果已经看到目标但目标没有居中，使用 30 度以内的小角度 `rotate` 微调方向。
7. 非必要情况下不使用 `move`；如果必须移动，只允许使用 1 米的小步 `move`。
8. 每次 `rotate` 或 `move` 后必须重新调用 `analyze_view`。
9. 如果 `analyze_view` 返回 `suggested_action=rotate_left_search`，或 `center_offset_x < 0` 表示目标在画面左侧，则只能调用 `rotate(direction="left", ...)`。
10. 如果 `analyze_view` 返回 `suggested_action=rotate_right_search`，或 `center_offset_x > 0` 表示目标在画面右侧，则只能调用 `rotate(direction="right", ...)`。
11. 生成 `rotate` 参数前，必须再次核对 `suggested_action`、`target_position` 和 `center_offset_x`，禁止因为口头推理或手误写反方向。

## 可调用工具

推荐工具顺序：

```text
analyze_view -> rotate -> analyze_view -> rotate -> analyze_view
```

## 安全约束

- 非必要情况下不要使用 `move`。
- 即使必须移动，也只允许小步移动 1 米。
- 不要在没有重新观察的情况下连续 `rotate` 或 `move`。
- 如果视觉结果明确提示左转或右转，实际调用的 `rotate(direction=...)` 必须与提示一致。
- 视觉结果不确定时，停止继续动作并说明原因。
- 飞行动作仍然必须遵守 HITL、语言介入和安全门。

## 失败处理

如果 `analyze_view` 失败，说明视觉分析失败，不继续飞行动作。

如果完成一圈粗搜索后仍未找到目标，向用户说明当前画面未发现目标，并等待下一步指令。

如果视觉结果提示的旋转方向与准备调用的 `rotate(direction=...)` 不一致，停止本次动作并重新分析方向，不要继续执行错误旋转。

## 反例

用户只是询问电池、飞行模式、当前位置、起飞或降落时，不使用这个 skill。

## 示例

用户：

```text
寻找红色椅子，并让无人机对准它
```

推荐工具调用顺序：

```text
analyze_view(target_description="红色椅子")
rotate(direction="left", degrees=90)
analyze_view(target_description="红色椅子")
rotate(direction="left", degrees=20)
analyze_view(target_description="红色椅子")
```

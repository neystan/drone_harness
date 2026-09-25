# Phase 8：Skills 设计文档

日期：2026-06-21

## 1. 目标

Phase 8 的目标是引入 Claude/Codex 风格的 `skills`，让 `drone_agent` 可以复用用户确认过的任务经验和工作流程。

第一版只做两件事：

- 支持项目内置 `drone_agent/skills/` 中的手写 `SKILL.md`。
- runtime 根据用户输入选择 0 或 1 个 skill。
- 将选中的 skill 正文注入本轮 LLM 上下文。
- skill 只指导 Agent 如何调用现有 `tools`，不新增执行入口。
- 提供内置 `skill_creator`，用于创建和校验标准格式的手写 skill。

## 2. 核心原则

`drone_agent` 是飞控 agent，不是通用文件操作 agent，所以 skill 体系必须比 nanobot、Codex、Claude Code 更保守。

Phase 8 遵守以下原则：

- `tools/` 仍然是唯一动作执行入口。
- `skills/` 只提供任务方法论、流程、约束和示例。
- skill 不能绕过 `tool_dispatcher.py`。
- skill 不能绕过 HITL、语言介入、超时和起飞前安全门。
- 第一版不支持 skill 自带可执行脚本。
- 第一版不支持从日志自动生成并启用 skill，但需要提供人工创建 skill 的 `skill_creator`。
- 第一版不支持 workspace skill 覆盖内置 skill。

## 3. Skill 与 Tool 的区别

`tool` 是 LLM 可以调用的原子动作。

例如：

- `takeoff`
- `move`
- `rotate`
- `analyze_view`
- `battery_status`

`skill` 是指导 LLM 如何完成某类任务的说明书。

例如：

- 视觉搜索目标时，先观察，再粗搜索，再精对准。
- 真机低高度试飞时，先查电池和飞行模式，再起飞到低高度。
- 工具失败或置信度低时，停止继续动作并向用户说明。

简单说：

```text
Skill 负责策略
Tool 负责动作
Safety 负责限制
Runtime 负责串联
```

## 4. 参考 nanobot 后的取舍

nanobot 的 skill 体系有几个值得借鉴的点：

- 每个 skill 是独立目录。
- 每个 skill 必须有 `SKILL.md`。
- `SKILL.md` 使用 YAML frontmatter 描述 `name` 和 `description`。
- 正文只在 skill 触发后注入上下文。
- 可以校验 skill 结构，避免 TODO、非法字段和目录名不一致。

但以下能力不适合 `drone_agent` Phase 8 直接照搬：

- 不使用 workspace skill 覆盖内置 skill。
- 不使用 `requires.env` 依赖环境变量。
- 不使用 `summary + path` 让 LLM 自己读取 skill 文件。
- 不启用 `scripts/`。
- 不打包 `.skill` 文件。

原因是 `drone_agent` 当前没有文件读取工具，LLM 不能靠路径自己打开 `SKILL.md`；同时飞控系统必须保持清晰的动作执行链路，不能让 skill 脚本成为第二条控制通道。

## 5. 推荐目录结构

Phase 8 建议增加：

```text
drone_agent/
  skills/
    __init__.py
    loader.py
    validator.py
    skill_creator.py
    visual-search/
      SKILL.md
    real-low-altitude-test/
      SKILL.md
  tools/
    skill.py
```

说明：

- `loader.py`：扫描和读取内置 skills。
- `validator.py`：校验 skill 结构和 frontmatter。
- `skill_creator.py`：根据用户提供的名称、描述、mode 和正文模板创建手写 skill 草稿，并复用 validator 校验。
- `tools/skill.py`：提供 `activate_skill` 工具，负责校验、HITL 和返回完整 skill 正文。
- `visual-search/`：内置视觉搜索流程 skill。
- `real-low-altitude-test/`：内置真机低高度试飞流程 skill。

第一版不增加：

```text
scripts/
assets/
package_skill.py
init_skill.py
```

## 6. SKILL.md 格式

第一版 `SKILL.md` 采用固定结构。一个 skill 由两部分组成：

- frontmatter：机器可读的索引信息，用于加载、过滤、选择和日志记录。
- Markdown 正文：模型可读的任务方法论，只在 skill 被选中后注入本轮上下文。

示例：

```yaml
---
name: visual-search
description: 当用户要求寻找目标、搜索目标、观察环境、判断目标是否出现在画面中，或让无人机对准某个目标时使用，指导 agent 分步完成视觉搜索和目标对准。
enabled: true
mode: ["sim", "real"]
---

# Visual Search

## 使用场景

## 工作流程

## 可调用工具

## 安全约束

## 失败处理

## 反例

## 示例
```

### 6.1 上下文注入边界

Phase 8 第一版采用三层结构：

```text
第 1 层：SYSTEM_PROMPT
  保留现有全局飞控约束

第 2 层：Skills Index
  注入所有已启用 skill 的 name + description
  让模型知道当前有哪些 skill 可以使用

第 3 层：Active Skill
  主 LLM 调用 activate_skill 后，由工具返回该 skill 的正文
```

也就是说：

- `name`、`description` 会组成全局 `skills index`，常驻在对话上下文中。
- `enabled`、`mode` 主要给代码使用，用于过滤 skill。
- 主 LLM 根据 `skills index` 决定是否调用 `activate_skill`。
- `activate_skill` 会校验 skill 是否存在、是否启用、当前 profile 是否允许，并触发 Y/N 人工确认。
- 用户确认后，`activate_skill` 才会把正文作为工具结果返回给 LLM。
- 未选中的 skill 不会把正文注入 LLM。
- 第一版不做 `always skill`，避免长期污染飞控 system prompt。

### 6.2 frontmatter 字段

`name`

- 必填。
- 必须和目录名一致。
- 只能使用小写字母、数字和连字符。

`description`

- 必填。
- 用于说明 skill 的作用和触发场景。
- 不允许保留 TODO 占位内容。

`enabled`

- 必填。
- `false` 时不参与选择。

`mode`

- 必填。
- 可选值：`sim`、`real`。
- 当前 profile 不匹配时不参与选择。

### 6.3 正文字段

正文是模型真正会读到的任务指导内容。

第一版建议固定包含：

`使用场景`

- 说明什么时候应该使用这个 skill。

`工作流程`

- 给出任务执行步骤。
- 应尽量写成简洁、有顺序的流程。

`可调用工具`

- 说明该 skill 推荐使用哪些现有 tools。
- 这里只是提示词约束，不改变真实 tool schema。

`安全约束`

- 写清楚哪些动作必须保守。
- 写清楚哪些情况下应停止继续执行。

`失败处理`

- 写清楚工具失败、超时、视觉低置信度、用户介入时如何响应。

`反例`

- 写清楚哪些请求不应该使用该 skill。

`示例`

- 给出典型用户请求和推荐工具调用顺序。

## 7. Skill 选择流程

第一版采用主 LLM 判断 + `activate_skill` 工具启用。

流程：

```text
用户输入
  -> 主 LLM 查看 Skills Index
  -> 如果需要某个 skill，调用 activate_skill(name)
  -> Python 校验 skill 是否存在、enabled、mode 是否匹配
  -> 触发 Y/N 人工确认
  -> 用户确认后返回 skill 正文
  -> 主 LLM 根据 skill 正文继续调用普通 tools
```

第一版不做向量检索，也不让 LLM 自己读取 skill 文件路径。

原因：

- 当前 skill 数量少。
- `activate_skill` 不需要为每轮对话额外增加一次 LLM router 调用。
- 飞控任务需要可解释的触发路径。

## 8. Skills Index 与 Active Skill

### 8.1 Skills Index

每轮调用 LLM 前，runtime 先把所有已启用 skill 的简短索引放进上下文。

格式建议如下：

```markdown
# Skills Index

- visual-search: 当用户要求寻找、搜索、观察或对准目标时使用。
- real-low-altitude-test: 当用户要求真机低高度试飞、悬停测试、低高度移动测试时使用。
```

这层只包含：

- `name`
- `description`

不包含：

- 全部 frontmatter
- 全部正文
- 示例和流程细节

作用：

- 让模型知道“当前系统有哪些 skill”
- 帮助模型理解 runtime 选中某个 skill 的语义
- 避免把所有 skill 正文长期塞进上下文

### 8.2 activate_skill

当前 `runtime.py` 会创建基础 system prompt：

```text
SYSTEM_PROMPT
```

如果主 LLM 需要使用某个 skill，必须先调用：

```text
activate_skill(name="visual-search")
```

工具会完成 Python 校验和 Y/N 人工确认。确认后，工具结果中返回 `skill_content`，主 LLM 再根据该内容继续调用其他 tools。

注意：

- 不修改全局 `SYSTEM_PROMPT` 常量。
- `skills index` 可以常驻，但只包含 `name + description`。
- `skills index` 第一版不包含文件路径，避免让模型绕过 `activate_skill` 直接读取 skill 文件。
- 每轮最多注入一个 skill。
- 只有命中的 `active skill` 才注入正文。
- skill 注入只影响 LLM 规划，不改变 tool schema。

### 8.1 为什么不采用 summary + path

nanobot 可以把所有 skill 摘要和文件路径放进上下文，让 agent 自己决定是否读取某个 `SKILL.md`。

`drone_agent` 第一版不采用这种方式。

原因：

- 当前 LLM 没有 `read_file` 工具。
- 只告诉模型路径没有实际作用。
- 飞控任务需要 runtime 明确选择 skill，而不是让模型自己找文件。

所以第一版由代码完成选择，再把选中的 `active skill` 正文直接注入本轮上下文。

## 9. 与现有安全机制的关系

Phase 8 不改变安全机制。

现有机制继续生效：

- `human_in_the_loop_for_flight_tools`
- `INTERRUPTED_BY_USER`
- `*_TIMEOUT`
- 飞行工具介入时 hover
- `takeoff()` 内最小起飞前安全门

skill 只能告诉 LLM “应该怎么做”，不能决定“是否绕过安全检查”。

如果 skill 内容和现有安全机制冲突，以现有代码安全机制为准。

## 10. 日志记录

Phase 8 通过工具调用日志记录 skill 启用。

示例：

```json
{
  "timestamp": "2026-06-21 20:30:10",
  "event_type": "tool_call",
  "tool_name": "activate_skill",
  "arguments": {"name": "visual-search"},
  "result": {"success": true, "skill_name": "visual-search"}
}
```

## 11. skill_creator

Phase 8 第一版需要提供 `skill_creator`，但它的定位是“人工创建手写 skill 的辅助工具”，不是自进化系统。

### 11.1 作用

`skill_creator` 负责：

- 根据用户提供的名称生成合法 skill 目录名。
- 创建标准 `SKILL.md` 模板。
- 填入基础 frontmatter。
- 创建必要的正文小节。
- 调用 `validator.py` 校验结果。

它不负责：

- 不读取历史日志自动总结经验。
- 不自动决定启用某个 skill。
- 不生成可执行 Python 脚本。
- 不修改飞控工具代码。

### 11.2 推荐使用方式

第一版建议提供一个 Python 函数和一个可选 CLI。

函数接口：

```python
create_skill(
    name: str,
    description: str,
    mode: list[str],
) -> Path
```

可选 CLI：

```text
python -m drone_agent.skills.skill_creator visual-search
```

CLI 第一版只做模板初始化，不需要复杂交互。

### 11.3 和自进化的关系

后续如果实现“从日志提炼 skill”，也应该复用 `skill_creator`。

流程应该是：

```text
日志分析器生成候选内容
  -> skill_creator 生成 draft SKILL.md
  -> 用户 review
  -> 用户确认
  -> enabled: true
```

当前 Phase 8 只实现其中的 `skill_creator 生成标准草稿`，不实现日志分析器。

## 12. 内置 Skill 建议

### 12.1 visual-search

适用场景：

- 用户要求寻找目标。
- 用户要求观察前方是否有某物。
- 用户要求对准某个目标。

核心流程：

```text
analyze_view
  -> 没看到目标：按 60 度大步 rotate 向左粗搜索一圈
  -> 看到目标但不居中：根据 suggested_action / target_position / center_offset_x 做 30 度以内小角度 rotate 微调
  -> 非必要不 move；必须靠近时只允许 1 米小步 move
  -> 每次动作后重新 analyze_view
```

安全约束：

- 当前前视相机水平视野约 90 度，粗搜索尽量持续向左旋转，避免来回切换方向。
- `found_target=false` 时，VLM 的 `suggested_action` 默认应为 `rotate_left_search`，和粗搜索策略保持一致。
- `found_target=true` 后才使用方向微调规则：`rotate_left_search` / `center_offset_x < 0` 只能左转，`rotate_right_search` / `center_offset_x > 0` 只能右转。
- 每次移动或旋转后必须重新观察。
- 非必要情况下不使用 `move`，必须移动时只允许小步移动 1 米。
- 视觉置信度低时应停止并说明。

### 12.2 real-low-altitude-test

适用场景：

- 真机低高度试飞。
- 用户要求起飞测试、悬停测试、低高度移动测试。

核心流程：

```text
battery_status
  -> flight_mode_status
  -> current_position_status
  -> takeoff 低高度
  -> hover / timer
  -> land
```

安全约束：

- 真机优先低高度。
- 起飞前必须确认状态。
- 异常时优先 hover 或 land。

## 13. 第一版不做的内容

Phase 8 不做：

- 不从日志自动提炼 skill。
- 不让 skill creator 自动从日志写入 `SKILL.md`。
- 不启用 skill 自带脚本。
- 不支持 skill 打包分发。
- 不支持 workspace 覆盖内置 skill。
- 不引入 multi-agent。
- 不引入 async runtime。

这些能力可以留到后续：

- Phase 8.2：从日志生成 skill 候选内容。
- Phase 8.3：用户确认后启用 draft skill。
- Phase 9：multi-agent 与异步 bus。

## 14. 实施顺序

建议按下面顺序实现：

1. 新增 `drone_agent/skills/` 包。
2. 新增 `validator.py`，校验 `SKILL.md`。
3. 新增 `skill_creator.py`，创建标准手写 skill 草稿。
4. 新增 `loader.py`，加载内置 skills。
5. 新增 `tools/skill.py`，提供 `activate_skill`。
6. 新增 `visual-search/SKILL.md`。
7. 新增 `real-low-altitude-test/SKILL.md`。
8. 修改工具 schema 和 registry，注册 `activate_skill`。
9. 修改 system prompt 和 Skills Index，引导主 LLM 先调用 `activate_skill`。
10. 更新 `PROJECT_ARCHITECTURE.md` 和 `DRONE_AGENT_SPEC.md`。

## 15. 验收标准

Phase 8 完成后应满足：

- 启动时能加载项目内置 skills。
- 无效 `SKILL.md` 会被清晰拒绝或跳过。
- `skill_creator` 能生成标准格式的 `SKILL.md` 草稿。
- 用户输入“寻找目标”时能选中 `visual-search`。
- 用户输入“真机低高度试飞”时能选中 `real-low-altitude-test`。
- 没有匹配 skill 的普通对话不注入 skill。
- 每轮最多注入一个 skill。
- 未选中的 skill 正文不会注入上下文。
- tool schema 不因 skill 改变。
- 飞行工具仍然触发 HITL、语言介入和安全门。
- 日志能看到本轮使用了哪个 skill。

## 16. 后续扩展

Phase 8.2 可以再考虑从日志生成候选 skill。

推荐流程：

```text
读取会话日志
  -> 发现重复任务模式
  -> 生成候选 SKILL.md
  -> 用户 review
  -> 用户确认
  -> 写入 draft skill
  -> 用户显式启用
```

关键限制：

- 自动提炼可以由 Agent 做。
- 正式启用必须由用户确认。
- 初期只生成说明型 `SKILL.md`。
- 如果后期支持脚本，脚本只能做离线分析，不能直接控制无人机。

# drone_harness

`drone_harness` 是基于 ROS 2、MAVROS 与 PX4 的单目标 RGB-D 飞行闭环项目。它从原 `drone_agent` 工作副本受控迁移而来；原仓库保持只读。当前按唯一保留的 [详细实施文档](docs/drone_harness-Phase1-单目标飞行闭环-详细实施文档.md) 分阶段实现。

> 当前阶段 3：按需 `observe(prompt)` 与前进时的新深度安全门已通过本地离线回归；真实仿真由用户后续验证，尚未宣称单目标闭环或最终停距验收。只在已抽测的 scene 9、13、21、24 使用仿真深度规则；scene 7 旧起点不用于新测试。真机自主飞行不在本阶段授权内。

## 包与入口

- Python/ROS 包：`drone_harness`
- 仿真入口：`drone_harness_sim`
- 真机入口：`drone_harness_real`（Phase 1 不进行真机自主飞行）
- profile：`drone_harness/config/profiles/sim.yaml` 与 `real.yaml`

示例设置见 `settings.example.json`。本地配置路径为 `~/.config/drone_harness/settings.json`，也可用 `DRONE_HARNESS_SETTINGS` 指向本地文件。API Key 不得提交到仓库。

Phase 1 只使用 `settings.json` 的 `llm` 配置作为唯一图片 VLM。可通过仅在当前进程设置的 `DRONE_HARNESS_LLM_API_KEY`、`DRONE_HARNESS_LLM_BASE_URL` 和 `DRONE_HARNESS_LLM_MODEL` 覆盖示例值；完整 `.../chat/completions` 地址也会归一化为 SDK 所需的基础地址。旧 `vlm`、`detector`、`tracker` 并行模型配置会被拒绝。无飞行接口探针见 `scripts/probe_multimodal_provider.py`，只使用内存中的合成图与假工具结果，不连接 ROS/PX4。

## 离线检查

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q
```

当前机器的系统 pytest 与自动加载的插件不兼容，故临时关闭插件自动加载。S1 的原样迁移基线为 27 通过、6 失败；这些失败来自旧飞行安全测试替身与现有 `flight_state()` 接口不匹配，后续安全阶段须保留等价断言并修复。

阶段 3 当前工作区的离线回归为 146 项通过；尚未进行真实仿真，也未验证 19 米位置指令的实际速度和超调。

Phase 1 不把模型的“候选完成”当作真实到达，也不把前视相机当作全向避障证明。S7 仿真前须完成深度语义核对、保守标定与隔离运行环境预检。

新任务先由同一个多模态 VLM 接收文字；模型调用 `observe(prompt)` 后，当前 RGB 与同号深度摘要才会交给它。飞行动作后不自动送图，提示词建议模型再次观察。仿真 `forward` 则**每次调用都另取新 RGB-D**，按 20 米决策视距、单次 19 米上限及深度规则计算实际前进距离；深度缺失或无效时不发布前进目标，只返回“未移动”及原因。深度几何计算预留 1 米**计划**净空，请求超限会被缩短并回报请求/指令距离。该余量不是位置 setpoint 的最终物理停距保证；原超时、人工介入和 PX4 安全交接仍有效。连续三次平移无位移会停，成功非零旋转会重置计数。详见[详细实施文档](docs/drone_harness-Phase1-单目标飞行闭环-详细实施文档.md)。

# drone_harness

`drone_harness` 是基于 ROS 2、MAVROS 与 PX4 的单目标 RGB-D 飞行闭环项目。它从原 `drone_agent` 工作副本受控迁移而来；原仓库保持只读。当前正在按 [设计规范](docs/drone_harness-Phase1-单目标飞行闭环-设计规范.md) 和 [详细实施文档](docs/drone_harness-Phase1-单目标飞行闭环-详细实施文档.md) 分阶段实现。

> S2 中间状态：旧检测、追踪和 skill 工具链已移除，`forward` 暂时拒绝执行。S3–S6 未完成前不要启动自主飞行入口；当前提交只用于离线测试。

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

Phase 1 不把模型的“候选完成”当作真实到达，也不把前视相机当作全向避障证明。S7 仿真前须完成深度语义核对、保守标定与隔离运行环境预检。

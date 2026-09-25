# S1 受控迁移与离线基线记录

> 2026-09-25；阶段状态：**本地迁移与 public 远端建仓完成，S1 待用户验收**。本记录不授权进入 S2。

## 来源与范围

- 只读来源：`/home/stan/AirVLN/drone_agent`，HEAD `d5fcb43754848b67a8d287e0f4b7f358e94fee60`，remote `https://github.com/neystan/drone_agent.git`。
- 迁移的是来源当前实际工作内容：87 个 Git 跟踪文件，加 1 个未跟踪测试 `test/test_llm_client.py`；其中包含现有 dirty 修改，并非仅复制 HEAD。
- 迁移前后，来源的 `git ls-files -co --exclude-standard -z | xargs -0 sha256sum | sha256sum` 聚合摘要均为 `902fd557c306cd9b4a9401edb456d0efad87d83403d3efb7d0e1b522321dce4e`；HEAD、remote 和 dirty 列表也保持不变。
- 另外从桌面复制了已讨论的 Phase 1 设计规范和详细实施文档到本仓库 `docs/`。
- 没有复制来源 `.git`、`.pytest_cache`、`__pycache__`、egg-info、真实 settings、密钥、日志、RGB-D 图像、仿真场景、数据集或 ROS/PX4 安装。`.gitignore` 在新仓库追加了运行图像/视频、数据集、ROS 记录和运行日志排除规则。
- 来源内容审查只在 `settings.example.json` 看到 `replace-with-your-...` 占位值；源代码里出现的 `api_key` 等字段名不是凭证。提交或推送前仍须再次审查候选文件。

## 仓库状态

- 本地目标：`/home/stan/AirVLN/drone_harness`，独立 `main` 仓库，不继承来源 `.git` 或历史。
- GitHub 登录账户已核对为 `neystan`；创建前同名仓库查询为 404。用户明确将可见性从原定 private 改为 public，并同意 GitHub CLI 临时目录浏览器授权。
- 已创建空仓库 `https://github.com/neystan/drone_harness`；GitHub API 核对 `full_name=neystan/drone_harness`、`visibility=public`、`private=false`、`size=0`。创建时没有推送文件，也没有覆盖原仓库。
- 公开候选为 91 个暂存文件；无未跟踪待提交文件，最大单文件 37,319 字节。暂存区格式检查通过；常见 GitHub/OpenAI/AWS 密钥、私钥头、Bearer 令牌和带账号密码的 URL 扫描均无命中。`settings.example.json` 只有 `replace-with-your-...` 占位值。上述扫描不能替代人工审查。

## 离线基线测试

| 命令 | 结果 | 说明 |
|---|---|---|
| `python -m pytest -q` | 未运行 | 当前 shell 无 `python` 命令 |
| `python3 -m pytest -q` | 环境错误 | 系统 pytest 6.2.5 自动加载本地 `anyio` 插件，缺少 `_pytest.scope` |
| `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q` | **27 通过、6 失败** | 6 例均在 `test/test_flight_safety_ack.py`：测试替身只有 `uav_is_in_air`，当前 `flight.py` 读取 `flight_state()`，故返回 `FLIGHT_STATE_UNKNOWN`；另有旧 pytest 不认识 `pythonpath` 配置的警告 |
| 用示例设置加载 sim/real profile 并导入两个 CLI 函数 | 通过 | 两 profile 分别解析为 `simulation`/`real`，未启动 ROS/PX4 |

这 6 例是从来源**原样迁移后的基线不一致**，不是 S1 新增回归；不得将它们写成通过。后续修改涉及原安全接口时，须修复测试替身并保留等价安全断言，再执行全量回归。S1 不修改飞行产品代码或为绿色结果删除测试。

额外基线事实：`scripts/drone_agent_sim` 在来源和副本中均可执行，`scripts/drone_agent_real` 在两处均没有 executable 位；两个 console-script 入口在打包声明中仍存在。该权限问题不能误写成迁移造成，后续打包/入口验收时应修正或明确使用 console-script 入口。

## S1 推送与验收门

1. 初始提交只纳入上述 91 个经审查的文件，推送到已核对的 public 远端；不使用强制推送。
2. 推送后核对远端提交、可见性，以及来源 HEAD、remote、dirty 列表和内容摘要仍与上述基线一致。
3. 清除临时 GitHub CLI 本地凭据；向用户报告远端链接、提交、测试结果和遗留的 6 例基线失败，等待 S1 验收后进入 S2。

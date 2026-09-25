# S2 旧链清理与统一命名验收

2026-09-25。本阶段只做离线代码与打包检查，不启动 ROS/PX4、仿真或真机入口。

## 变更与保留

- 新仓库的 Python/ROS 包、配置目录、环境变量和 sim/real 双入口统一为 `drone_harness`。原 `/home/stan/AirVLN/drone_agent` 与 S1 历史迁移记录不改。
- 模型注册表只剩 `takeoff`、`forward`、`rotate`、`land`；`forward` 在 S5 完成前固定返回 `FEATURE_NOT_READY`，不会触达 controller。
- 移除旧的独立视觉分析、DINO 检测、SAM2 追踪、鼠标选点、skills、模型可见状态/拍照/计时/任意位移工具，以及只服务于追踪叠图的相机预览脚本。与这些旧功能绑定的 `test_tracker_runtime_safety.py` 被移除，并由四工具表面与前进占位拒绝测试替代。旧架构分期文档可在 S1 Git 历史恢复，当前文档只保留本项目设计和实施记录。
- 保留 `Px4Controller`、底层 `flight.move`、应急悬停/RTL、状态读取、消息总线、ROS executor 与安全交接。AirSim launch 只启动相机桥接；真机 profile 的 AirSim RGB topic 清空，避免误用。
- S1 的 6 个飞行安全测试失败来自替身缺少现有 `flight_state()` 与高度查询接口。本阶段仅补齐替身，并保持原 ACK、超时和安全交接断言，没有修改这些飞行产品实现。

## 检查结果

| 检查 | 结果 |
|---|---|
| `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q` | 33 通过；已移除对旧系统 pytest 无效的 `pythonpath` 选项 |
| `DRONE_HARNESS_SETTINGS=.../settings.example.json python3 -c '...prepare_runtime("sim"); prepare_runtime("real")'` | 两 profile 均可解析，`ros_started=False` |
| `python3 setup.py --name` | `drone_harness` |
| `python3 -m compileall -q drone_harness` | 通过 |
| `rg` 搜索运行代码中的旧包名及旧视觉/skill 链 | 仅 README、迁移事实与断言旧路径不存在的测试中保留历史名称；运行代码无引用 |

来源仓库 HEAD 仍为 `d5fcb43754848b67a8d287e0f4b7f358e94fee60`，工作文件聚合 SHA-256 仍为 `902fd557c306cd9b4a9401edb456d0efad87d83403d3efb7d0e1b522321dce4e`；其 dirty 文件未清理。

S2 不表示闭环可以飞行。S3 必须证明 RGB-D 时间配对、深度语义与数值上限；S5 前 `forward` 必须继续拒绝。

# drone_agent 项目架构文档

本文档描述当前 `/download/drone_agent` 的实际目录结构，并说明每个目录、每个文件的职责。

## 1. 项目定位

`drone_agent` 同时承担两个角色：

- GitHub 主仓
- ROS2 `ament_python` 包源码目录

运行入口有两种：

- 直接命令：`drone_agent_sim`、`drone_agent_real`
- ROS2 命令：`ros2 run drone_agent drone_agent_sim`、`ros2 run drone_agent drone_agent_real`

## 2. 顶层目录与文件

### 2.1 项目自有目录

| 路径 | 作用 |
| --- | --- |
| `docs/` | 项目文档目录。当前包含历史设计文档归档和本架构文档。 |
| `drone_agent/` | 主 Python 包，包含配置、运行时、消息总线、PX4 控制、工具、视觉、日志等核心代码。 |
| `launch/` | ROS2 launch 文件，主要来自旧 ROS2 包壳，供相机/示例链路启动参考。 |
| `resource/` | ROS2 `ament_python` 包索引资源目录。 |
| `rviz/` | RViz 配置文件目录，主要用于旧链路的可视化配置。 |
| `scripts/` | 可执行包装脚本目录，提供 `drone_agent_sim` 和 `drone_agent_real`。 |
| `tests/` | 本地单元测试目录。当前已加入 `.gitignore`，默认不上传。 |

### 2.2 顶层文件

| 路径 | 作用 |
| --- | --- |
| `.gitignore` | Git 忽略规则，忽略 `settings.json`、`tests/`、缓存、日志等本地文件。 |
| `README.md` | 项目使用说明，包含配置方式、ROS2 集成方式和启动命令。 |
| `package.xml` | ROS2 包元数据，声明 `drone_agent` 为 `ament_python` 包及其运行依赖。 |
| `pyproject.toml` | 独立 Python 包构建配置，支持 `pip install -e .` 和 console scripts。 |
| `settings.example.json` | 模型配置模板文件，示例展示 `llm` 和 `vlm` 的 `api_key/base_url/model` 字段。 |
| `settings.json` | 本地私有模型配置文件，实际运行读取它。已被 `.gitignore` 忽略。 |
| `setup.cfg` | ROS2 Python 安装脚本路径配置。 |
| `setup.py` | ROS2 `ament_python` 打包入口，注册资源文件、launch、rviz、启动脚本和内置 skill 文件。 |

## 3. `docs/` 文档目录

| 路径 | 作用 |
| --- | --- |
| `docs/PROJECT_ARCHITECTURE.md` | 当前文档，解释项目实际架构。 |
| `docs/legacy_specs/` | 历史设计文档归档目录，不直接参与运行。 |
| `docs/legacy_specs/DRONE_AGENT_SPEC.md` | 早期架构规格说明。 |
| `docs/legacy_specs/MVP_DESIGN.md` | 早期 MVP 设计文档。 |
| `docs/legacy_specs/UAV-Claw-MVP-Design.md` | 更早的抓取/任务相关设计文档。 |

## 4. `drone_agent/` 主 Python 包

### 4.1 包根文件

| 路径 | 作用 |
| --- | --- |
| `drone_agent/__init__.py` | 包标记文件。 |
| `drone_agent/__main__.py` | `python -m drone_agent` 的入口，默认转到仿真模式。 |
| `drone_agent/cli.py` | 命令行入口层，提供 `main_sim()`、`main_real()`，负责把 profile 名交给 runtime。 |

### 4.2 `drone_agent/config/`

职责：管理运行配置，分离飞行 profile 和模型 settings。

| 路径 | 作用 |
| --- | --- |
| `drone_agent/config/__init__.py` | 配置子包标记文件。 |
| `drone_agent/config/loader.py` | 配置加载器。读取 `sim.yaml/real.yaml` 和项目根目录 `settings.json`，组装 `RuntimeProfile`。 |
| `drone_agent/config/schema.py` | 配置数据结构定义，包含 `RuntimeProfile`、`RosConfig`、`StorageConfig`、`ProviderConfig`、`VlmConfig`、`SafetyConfig`。`SafetyConfig` 当前还包含 `pre_takeoff_gate_enabled`、`require_battery_status_for_takeoff`、`min_battery_percent_for_takeoff`、`require_px4_status_ready_for_takeoff`。 |
| `drone_agent/config/profiles/` | 运行 profile 目录，描述仿真/真机的 ROS、存储和安全差异。 |
| `drone_agent/config/profiles/__init__.py` | profile 子目录的包标记文件，便于 setuptools 正确分发 YAML 配置。 |
| `drone_agent/config/profiles/sim.yaml` | 仿真 profile，配置 AirSim 相机 topic、MAVROS namespace、图片目录、日志目录和仿真安全阈值。 |
| `drone_agent/config/profiles/real.yaml` | 真机 profile，配置真机节点名、存储路径和更严格的安全阈值；MAVROS 默认 namespace 为 `/mavros`，真机相机 topic 需按设备实际配置。 |

说明：

- `settings.json` 负责模型配置：`api_key`、`base_url`、`model`
- `sim.yaml/real.yaml` 只负责飞控与运行环境配置

### 4.3 `drone_agent/bus/`

职责：管理用户输入消息的传递、独立输入终端接入和语言介入基础设施。

| 路径 | 作用 |
| --- | --- |
| `drone_agent/bus/__init__.py` | bus 子包导出，统一暴露 `InputServer`、`MessageBus` 等对象。 |
| `drone_agent/bus/queue.py` | 线程安全的同步消息队列封装。 |
| `drone_agent/bus/message_bus.py` | `MessageBus` 与 `UserMessage` 定义，提供用户消息发布/消费接口。 |
| `drone_agent/bus/input_server.py` | 主进程内的本地输入服务，接收独立输入终端发送的文本并写入 `MessageBus`。 |
| `drone_agent/bus/input_terminal.py` | 独立输入终端客户端，负责显示 `you>` 并把用户输入回传主进程。 |
| `drone_agent/bus/intervention.py` | 语言介入检测与中断结果构造，必要时触发悬停保护。 |

### 4.4 `drone_agent/runtime/`

职责：连接 LLM、工具系统和 ROS2 运行时。

| 路径 | 作用 |
| --- | --- |
| `drone_agent/runtime/__init__.py` | 运行时子包标记文件。 |
| `drone_agent/runtime/agent_loop.py` | 单轮 Agent 对话循环，负责调用 LLM、执行工具调用、处理中断条件。 |
| `drone_agent/runtime/runtime.py` | 运行总控。负责加载 profile、创建 ROS2 executor、创建 `Px4Controller`、创建 LLM client、创建 `MessageBus`/`InputServer`、加载 skills 并启动交互 loop。 |
| `drone_agent/runtime/safety.py` | Agent 侧安全判定逻辑，例如飞行工具是否需要 Y/N 人工确认，以及哪些结果会直接结束当前轮。 |
| `drone_agent/runtime/task_state.py` | 任务状态数据结构、状态转移方法和彩色终端状态行格式化。 |
| `drone_agent/runtime/terminal.py` | 根据当前系统环境自动打开独立输入终端，并构造 `python -m drone_agent.bus.input_terminal` 启动命令。 |
| `drone_agent/runtime/tool_dispatcher.py` | 工具分发层。把模型输出的 tool call 解析后路由到具体工具处理函数。 |

### 4.5 `drone_agent/llm/`

职责：管理大模型接入与系统提示词。

| 路径 | 作用 |
| --- | --- |
| `drone_agent/llm/__init__.py` | LLM 子包标记文件。 |
| `drone_agent/llm/client.py` | 根据 `RuntimeProfile.llm` 创建 OpenAI-compatible 文本模型客户端。 |
| `drone_agent/llm/prompts.py` | 存放系统提示词 `SYSTEM_PROMPT`，约束 agent 如何调用飞行、状态、视觉工具。 |

### 4.6 `drone_agent/logging/`

职责：记录任务和工具调用日志。

| 路径 | 作用 |
| --- | --- |
| `drone_agent/logging/__init__.py` | 日志子包标记文件。 |
| `drone_agent/logging/task_log.py` | 以 JSONL 格式记录 agent 消息、工具调用结果和任务状态。 |

### 4.7 `drone_agent/skills/`

职责：管理 Claude/Codex 风格的说明型 skills。skill 只指导 Agent 如何使用现有 tools，不新增飞控执行入口。

| 路径 | 作用 |
| --- | --- |
| `drone_agent/skills/__init__.py` | skills 子包导出文件。 |
| `drone_agent/skills/context.py` | 构造全局 `skills index` 和 skill 终端显示内容。 |
| `drone_agent/skills/loader.py` | 扫描并加载项目内置 `SKILL.md`。 |
| `drone_agent/skills/skill_creator.py` | 创建标准格式的手写 skill 草稿，并复用 validator 校验。 |
| `drone_agent/skills/validator.py` | 校验 skill 目录、frontmatter 和允许的根目录内容。 |
| `drone_agent/skills/visual-search/SKILL.md` | 视觉搜索目标的任务方法论。 |
| `drone_agent/skills/real-low-altitude-test/SKILL.md` | 真机低高度试飞的任务方法论。 |

### 4.8 `drone_agent/px4/`

职责：封装 MAVROS 交互和底层控制能力，并保持原有 `Px4Controller` 类和目录路径不变。

| 路径 | 作用 |
| --- | --- |
| `drone_agent/px4/__init__.py` | PX4 子包标记文件。 |
| `drone_agent/px4/controller.py` | `Px4Controller(Node)` 实现。订阅 `/mavros/state`、`/mavros/local_position/pose`、`/mavros/battery` 和 `/mavros/extended_state`，通过 MAVROS service 发送命令，通过 `/mavros/setpoint_raw/local` 持续发布位置 setpoint，并记录状态接收情况。 |
| `drone_agent/px4/frame.py` | 坐标系和角度工具，例如 body 坐标到 NED 坐标转换。 |
| `drone_agent/px4/status.py` | PX4 状态字段解析与可读化辅助函数。 |
| `drone_agent/px4/topics.py` | 兼容旧状态常量和 MAVLink 命令结果名称定义。 |

### 4.9 `drone_agent/tools/`

职责：LLM 可调用工具层。

| 路径 | 作用 |
| --- | --- |
| `drone_agent/tools/__init__.py` | 工具子包标记文件。 |
| `drone_agent/tools/flight.py` | 飞行动作工具实现，如起飞、降落、返航、悬停、旋转、移动、计时，并在长时动作中检查语言介入。当前 `takeoff()` 还会执行最小真机起飞前检查。 |
| `drone_agent/tools/perception.py` | 感知工具实现，如拍照和视觉分析，负责从 controller 取最新图像并调用视觉模块。 |
| `drone_agent/tools/registry.py` | 工具注册表，定义工具名、schema、handler 之间的映射，并定义包含 `message_bus`/`task_state` 的 `ToolContext`。 |
| `drone_agent/tools/schemas.py` | Function Calling 的工具 schema 定义。 |
| `drone_agent/tools/skill.py` | skill 启用工具，负责校验 skill、触发 Y/N 人工确认，并把完整 skill 正文返回给主 LLM。 |
| `drone_agent/tools/status.py` | 状态查询工具，如当前位置、电池状态、飞行模式状态。 |

### 4.10 `drone_agent/vision/`

职责：视觉模型与图像文件处理。

| 路径 | 作用 |
| --- | --- |
| `drone_agent/vision/__init__.py` | 视觉子包标记文件。 |
| `drone_agent/vision/image_store.py` | 保存拍照结果和分析帧到磁盘。 |
| `drone_agent/vision/vlm.py` | 创建视觉模型客户端、构造视觉提示词、编码图像、调用 VLM、解析和归一化视觉结果；未发现目标时默认建议向左粗搜索，发现目标后再根据位置偏移给出微调方向。 |

## 5. `launch/` ROS2 launch 目录

这些文件主要是仿真辅助启动文件，不是当前 `drone_agent_sim` / `drone_agent_real` 主入口。

| 路径 | 作用 |
| --- | --- |
| `launch/lesson3.launch.py` | 旧链路的 ROS2 launch 文件。 |
| `launch/lesson6.launch.py` | 旧链路的 ROS2 launch 文件。 |
| `launch/takeoff_camera.launch.py` | 仅启动 AirSim ROS bridge 和仿真相机预览；MAVROS 必须独立启动，真机不使用此 launch。 |

## 6. `resource/` ROS2 资源目录

| 路径 | 作用 |
| --- | --- |
| `resource/drone_agent` | ROS2 `ament_index` 识别包名 `drone_agent` 的资源标记文件。 |

## 7. `rviz/` 配置目录

这些文件主要服务于旧的可视化调试流程。

| 路径 | 作用 |
| --- | --- |
| `rviz/depth_cloud.rviz` | 深度点云显示配置。 |
| `rviz/image_lidar.rviz` | 图像与雷达联合显示配置。 |
| `rviz/lesson6.rviz` | 旧 lesson6 场景的 RViz 配置。 |

## 8. `scripts/` 启动脚本目录

| 路径 | 作用 |
| --- | --- |
| `scripts/drone_agent_sim` | 仿真模式包装脚本，最终调用 `drone_agent.cli:main_sim`。 |
| `scripts/drone_agent_real` | 真机模式包装脚本，最终调用 `drone_agent.cli:main_real`。 |
| `scripts/camera_view_sim` | 仿真前视相机预览节点，供 `takeoff_camera.launch.py` 等旧 launch 链路启动 OpenCV 预览。 |

## 9. `tests/` 本地测试目录

`tests/` 当前保留在本地，但默认不上传。它主要用于回归验证，不参与生产运行。

### 9.1 `tests/unit/`

| 路径 | 作用 |
| --- | --- |
| `tests/unit/.gitkeep` | 空目录占位文件。 |
| `tests/unit/test_cli.py` | 测试 CLI 入口的 profile 路由和配置错误输出。 |
| `tests/unit/test_cli_runtime_mode.py` | 测试 CLI 与 runtime 模式之间的接线。 |
| `tests/unit/test_config_loader.py` | 测试 profile + settings 配置加载和校验。 |
| `tests/unit/test_llm_prompts.py` | 测试系统提示词中的关键约束是否存在。 |
| `tests/unit/test_px4_controller_source.py` | 对 `Px4Controller` 源码结构做静态检查。 |
| `tests/unit/test_px4_frame.py` | 测试坐标与角度转换函数。 |
| `tests/unit/test_px4_status.py` | 测试 PX4 状态解析逻辑。 |
| `tests/unit/test_runtime.py` | 测试 runtime 的 profile 准备逻辑。 |
| `tests/unit/test_safety.py` | 测试安全判定逻辑。 |
| `tests/unit/test_task_log.py` | 测试 JSONL 日志输出逻辑。 |
| `tests/unit/test_tool_dispatcher.py` | 测试工具分发、参数解析和错误处理。 |
| `tests/unit/test_tools_registry.py` | 测试工具注册表完整性和部分工具行为。 |
| `tests/unit/test_tools_schemas.py` | 测试 Function Calling schema 的结构。 |
| `tests/unit/test_vision_image_store.py` | 测试图片保存逻辑。 |
| `tests/unit/test_vision_vlm.py` | 测试视觉结果归一化、JSON 提取和建议动作推导。 |

## 10. 本地生成或非架构核心目录

这些目录存在于当前工作区，但不是项目源码架构的一部分：

| 路径 | 作用 |
| --- | --- |
| `.git/` | Git 元数据目录。 |
| `.pytest_cache/` | pytest 运行缓存。 |
| `__pycache__/`、`drone_agent/**/__pycache__/`、`tests/**/__pycache__/` | Python 字节码缓存。 |

## 11. 当前建议的阅读顺序

如果要快速理解项目，建议按下面顺序读：

1. `README.md`
2. `settings.example.json`
3. `drone_agent/cli.py`
4. `drone_agent/runtime/runtime.py`
5. `drone_agent/runtime/terminal.py`
6. `drone_agent/bus/message_bus.py`
7. `drone_agent/bus/intervention.py`
8. `drone_agent/skills/loader.py`
9. `drone_agent/tools/skill.py`
10. `drone_agent/config/loader.py`
11. `drone_agent/px4/controller.py`
12. `drone_agent/tools/registry.py`
13. `drone_agent/tools/flight.py`
14. `drone_agent/tools/perception.py`
15. `drone_agent/vision/vlm.py`

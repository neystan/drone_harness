# S6 单一多模态 VLM 接线与无飞行兼容性探针

> 本文件末尾的“S7 前限制”仅描述 S6 当时的代码状态；S7 已移除模型思考时的 A 帧龄执行期限，不再取 B 重算或增加运动中深度守卫。现行边界见[唯一详细实施文档](drone_harness-Phase1-单目标飞行闭环-详细实施文档.md)和[S7 状态](S7_ACCEPTANCE_STATUS.md)。

2026-09-25。本阶段没有启动 ROS、PX4、AirSim 或飞行入口，也没有执行模型提出的任何飞行动作。

## 接线与配置

- 只保留一个 `llm` provider，图片理解、深度规则阅读、规划和原生 function calling 均走现有 `OpenAI` 客户端与 `chat.completions.create`。移除未使用的独立 `vlm`、`detector`、`tracker` profile 类和加载路径；旧并行模型设置直接报错，不静默恢复第二模型。实例运行可通过仅当前进程的环境变量覆盖 API Key、基础地址和模型，不写入版本控制。
- 兼容用户给出的完整 `.../chat/completions` URL，客户端实际使用去掉该尾缀的基础地址；设置 20 s 请求超时、禁自动重试。图片由内存 JPEG data URL 发给同一请求，日志只留观测元数据。模型/响应异常停止，空中且仍在 OFFBOARD 时先确认 PX4 悬停；不退回无图文本规划。
- 脚本 `scripts/probe_multimodal_provider.py` 用两张 32×32 非敏感合成色块、四个真实 schema、模型提出但**绝不执行**的 `rotate` 假调用及假工具结果测试协议续聊。密钥由隐藏回显的终端输入读取，只在进程内存中使用；脚本只打印工具名称/数量、结束原因和耗时，不保存密钥、请求图片或原始回复。

## 核验

- 智谱官方公开资料确认 `GLM-5.3-Flash` 是原生多模态模型并具工具使用能力：<https://autoclaw.z.ai/blog/model/glm-5.3-flash/>。官方文档未直接证明指定端点支持本项目组合协议，因此实际接口探针仍是必要条件。
- 在用户指定的官方端点与 `glm-5.3-flash` 上，首个“图片 + 文本 + 四工具”请求成功返回**一个** `rotate` tool call，约 7.87 s；追加假工具结果和第二张新图后，请求成功返回文本且零 tool call，约 6.64 s。无飞行、无真实相机、无外部图像保存。密钥未写入仓库或日志。
- `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q`：85 通过；`python3 -m compileall -q drone_harness scripts/probe_multimodal_provider.py` 通过。测试覆盖单模型配置、旧多模型拒绝、完整 URL 归一化、同请求图文和四 schema、服务异常悬停与畸形响应零动作。

## S7 前的实际限制

本次服务往返约 7–8 s，远大于当前 sim profile 的 1 s 观测时效。现有执行门会因此拒绝模型提出的 `forward`，这属于预期的安全停止，**不能通过把运动中最长盲行时间一起放宽到 8 s 来伪装为通过**。S7 需要先在仿真中测服务延迟、相机/位姿更新和实际速度，评估是否可在模型决策后以最新 RGB-D 和静止位姿重新核验，且仍保持运动中守卫的短时效；若无法安全证明，则不放行自主前进。

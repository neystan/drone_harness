# S7 故障回放、仿真标定与验收状态

2026-09-25 初记，2026-09-26 更新。状态：**S7 离线修改已完成，真实仿真待用户执行；未开始单目标自主前进，未宣称目标到达或 S7 验收通过。** 下文旧停止记录保留时间背景，以文末“最新执行约定”为当前配置准绳。

## 按旧交接文档第 0、11 节的只读预检

- 环境仓库 `/home/stan/AirVLN_ws/AirVLN-theta-star`：HEAD `2fb19e66263a8b20bbe203550da7652d49544f75`，分支 `codex/theta-star-planner`。原有修改 `docs/handoffs/2026-09-14-airvln-theta-star-dagger-smoke-handoff.md`；原有未跟踪 `docs/aerialvln-s-px4-adapter.md`、`docs/aerialvln-s-test-cases.md`、`docs/handoffs/2026-09-22-aerialvln-s-px4-drone-agent-evaluation-handoff.md`、`tools/aerialvln_safe_move.py`，均未清理。
- 只读来源 `/home/stan/AirVLN/drone_agent`：HEAD `d5fcb43754848b67a8d287e0f4b7f358e94fee60`，原有 10 个修改文件与 1 个未跟踪测试；文件聚合 SHA-256 仍为 `902fd557c306cd9b4a9401edb456d0efad87d83403d3efb7d0e1b522321dce4e`，remote 仍是 `https://github.com/neystan/drone_agent.git`。旧项目未被编辑、提交或清理。
- 官方场景 7/9/13/21/24、AerialVLN-S 四个 split JSON、PX4 可执行文件、MAVROS、AirSim ROS2 overlay 均存在。启动前端口 4560/41451/14030/14280/14540/14580 均空闲，无相关旧进程。旧 `val_unseen` scene7 case489 成功报告位于 `/home/stan/AirVLN_ws/runs/aerialvln-px4/20260920-144207-960213-scene7-3NPFYT4IZL2P/evaluation.json`；其 SR=1、OSR=1 只证明旧流程，**不是本项目验收**。

## 本项目已得到的证据

- S1–S6 的本地阶段提交分别为 `2ac3b29`、`669f450`、`b5bbce1`、`2ceb783`、`a011bdc`、`708e35f`。新仓库 `drone_harness` 的入口、两个 profile、四个工具和单 VLM 接线已验证。GitHub public 远端当前仍在 S1 `2ac3b29`；S2–S6 未推送，不能以本地提交冒充远端完成。
- 按用户确认的 S7 静态场景合同，`forward` 仅使用已绑定 A 的深度有效性/数值上限与 profile 限额；模型思考或人工确认时间不触发 A 帧龄拒绝，不获取 B 或重算 B 的深度。撤除新增的发布前/运动中深度守卫和专用中断分支；仍保留原 `move` 的位置、飞行状态、超时、人工介入和 PX4 交接。只为短步 `forward` 保留严格到达容差，避免原 0.3 m 容差在起点误判完成。HITL 确认采用独立的 120 s 交互期限，批准后比较 A 绑定、上限与当前飞控状态，不将观测帧龄当作人工期限。
- `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q`：**103 passed**；`python3 -m compileall -q drone_harness test` 与 `git diff --check` 通过。固定回放覆盖取 A 时旧 RGB/无效深度、近障与上限、模型超时/坏回复/多动作、动作后无新 RGB；另覆盖有效 A 等待超过 7 s 仍可前进、无额外 B、真机四动作 HITL/交互超时、原 `move` 超时/人工中断及悬停未确认交接。撤除的守卫测试已改成当前合同的回归断言，没有用删除测试冒充通过。模拟器未参与本轮代码调整的离线回归。
- ROS overlay 构建并解析到新包 `/home/stan/AirVLN/drone_harness_release/install/drone_harness`。仿真 Scene RGB 为 `bgr8`，DepthPerspective 为 `32FC1`，内参 `fx=fy≈298.4048`、分辨率 640×480；RGB/深度采集时间差约 3 ms，MAVROS 已连接。
- 选定 `val_unseen` scene7 case489，唯一受控运行目录 `/home/stan/AirVLN/drone_harness_runtime/s7_runs/20260925-210325-796282-scene7-3NPFYT4IZL2P/`。`preflight.json` 中初始位姿误差约 0.539 m，在 1.0 m 容差内；RGB 可读。受控**仿真**起飞 1.0 m、观测、降落成功：空中高度约 0.906 m，降落后 MAVROS `landed_state=1`。没有执行自主 `forward`。
- 在地面从 ROS 深度及 AirSim 原始 RPC 各读取 640×480 深度，原始最大值均为 **1.0**，原始值中约 **65.06%** 恰好为 1.0；直接请求 `DepthPlanar` 仍以 1.0 封顶（约 67.36% 像素）。平移相机时深度像素会更新；临时标靶没有形成可定位的深度响应，所以无法由此证明 1.0 是“恰好 1 m”或“至少 1 m”。标靶已在临时仿真中销毁，相机位姿已恢复，随后仿真进程均收停；端口再次为空闲。原始统计及停止判据见运行目录 `calibration_stop.json`。官方 AirSim 文档的 `DepthPerspective` 设计语义不能替代当前资产的实测标定。
- 另在独立 scene7 进程中将相机相对机体抬高 0、0.05、0.10、0.20 m，不移动机体：画面第 400 行中心像素的原始深度依次为 0.4351、0.5449、0.6587、0.8823；第 320 行从 0.7998 变为精确 1.0。地面区域的增量与米制透视射线的预测量级一致，**支持**小于 1.0 的深度近似为米制、1.0 为当前资产的封顶值；但绝对误差和所有饱和像素作为 1 m 下界的合同未获独立证实，不能将推断当成全图标定。试后相机配置恢复。
- 在相同受控仿真中，用原 `move` 位置 setpoint 做非 VLM 短步标定：请求 0.10 m，峰值前移约 0.129 m、峰值采样速度约 0.122 m/s；另请求 0.30 m，在前移约 0.113 m 时触发原 `request_confirmed_hover`，PX4 确认 `AUTO.LOITER` 约需 0.601 s，触发后仍额外前移约 **0.196 m**，采样峰值速度约 0.324 m/s。PX4 当前 `MPC_XY_VEL_MAX=1.5 m/s`；上述单次采样不是可审计的最坏速度/刹停界，不能拿它们直接把 25 m 制动余量调小。两次试验均已降落并核对 `landed_state=1`，随后收停进程，端口为空闲。
- 真实 `glm-5.3-flash` 无飞行探针已经证明同一请求可接图片、摘要和四工具，下一请求可接假工具结果及新图；两次响应约 7.87 s 和 6.64 s。S7 已取消用 `max_frame_age_s=1.0` 限制模型思考或审批；该阈值仅用于取 A/C 时判断采集帧新鲜度。该探针没有触达飞控，不证明单目标闭环成功。
- 在另一次独立的默认相机配置检查中，`CameraDepth1` 与默认 `0`、`1`、`2`、`front_center` 的原始深度仍均以数值 1.0 封顶；随后请求未配置的 `front_0` 发生超时，场景进程以 Signal 11 退出。该次没有飞行，PX4 已收停、端口复查为空；它不构成量程标定通过的证据。

## 保守处理与未达成项

截至 2026-09-25，仿真 `depth_semantics` 已降为 `unverified`，深度规则恒给 `forward_max_m=0`，程序安全门零前进；真机配置本来就未验证。彼时超量程像素视为未知。这个更改是**停止条件的实现**，不是深度标定通过。

2026-09-26 按用户确认把后续 `val_unseen` 测试范围限定为 scene **9、13、21、24**；scene 7 的原始深度在旧起点封顶 1.0，不再选作新测试，但资产、数据集条目及旧报告均保留。隔离 `ComputerVision` 原始 RPC 在四场景各一处起点的向下深度中心随相机升高到约 25–40 m，说明这些样本没有 scene 7 的 1.0 截断；前视 scene 9、13、21 含大量 >20 m 远景，scene 24 的该样本前视全图在 20 m 内。临时证据：`/home/stan/AirVLN/drone_harness_runtime/s7_depth_probe.tAuyE9/probe_results.md`。这是抽样，不是所有起点、ROS 同帧或飞行验收。

彼时程序决策视距保持 `depth_max_m=20.0`。离线深度规则已取消“全图每像素都在 `(0,20] m`”的硬门：仅在输入已被声明并标定为透视射线米制时，将有限正的 >20 m 读数按 20 m 净空下界计算；NaN/Inf/零值仍未知，进入飞行通道则拒绝。640×480 相机的近场通道投影可能覆盖全图，故真正缺测的像素即使在画面角落也可能被拒绝；这与有可信远值的天空/远景不同。在当时尚须以已知距离靶标确认 1、3、5、10、20 m 的误差及远值/无命中编码，不能把 AirSim 文档或隔离抽样当成完整语义验收。该次烟测时仿真 profile 仍为 `depth_semantics: unverified` 且制动余量未标定，故**当时自主前进上限为零**，没有开始闭环飞行。

本轮离线验证：`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q` 为 **106 passed**；`python3 -m compileall -q drone_harness test`、`git diff --check` 通过。新增断言覆盖可信 >20 m 读数只能给出 20 m 下界、640×480 仿真相机内参下的远景、近障仍收紧上限、非通道角落缺测可容忍而通道缺测拒绝；sim/real profile 静态加载均保持深度 `unverified`。这些测试没有启动 AirSim、PX4 或真实飞行。

2026-09-26 继续完成四场景各一处起点的隔离已知距离薄板测试：1、3、5、10、20 m 各重复 3 次，`DepthPerspective` 中央 5×5 像素中位数均与标称值相符；`DepthPlanar` 最大误差为 scene 24 的 20 m 档约 0.01563 m。scene 21 再在真正 PX4 + ROS 桥的标准 640×480、无像素格式覆盖配置下复测五档，每档 9–14 帧的中心区域数值也与标称相符。朝天空看会返回 65504 或约 1.63 万米等**有限正大数**，但仅凭此不能区分远景几何、远裁剪与无命中编码；本轮原始样本没有 NaN/Inf/0/负值。对本次静态场景，这支持把可信 >20 m 视作“20 m 内未见表面”的下界，但不是对所有起点、所有资产或实体相机的普遍承诺。详细方法、样本和限制见本地 `/home/stan/AirVLN/drone_harness_runtime/s7_range_test.h2wNe9/results.md`。

真正 PX4 仿真下，四场景均有 640×480 的 `bgr8` RGB、`32FC1` 深度与 CameraInfo。scene 13、21 的“直接取最新帧”出现偶发错配；复用实际 `wait_for_snapshot` 后各连续 30/30 次取得有效新观测，最长分别 0.351/0.397 s。scene 21 标准场景配置**不加**临时像素格式覆盖仍有大量 >20 m 原始读数，等待配对 30/30 成功，因此暂不需要改标准 AirSim 像素格式。scene 24、13、21 的 harness 地面烟测均收到 MAVROS 状态、位姿、RGB-D，并生成同一观测的文字摘要与 JPEG 消息；没有设置移动目标。bridge 初始化会主动请求 PX4 解锁，随后 PX4 自动解除；这属于保留下来的上游行为，不能把本次烟测描述成“从未发生过解锁请求”。

用户当时把目标收窄为“系统正常运行的证据”，不再要求把单次悬停后约 0.196 m 的额外前移推广为最坏超调，也不要求独立证明目标到达。故该次烟测不把目标到达列为已完成项，也不因未到达而冒称测试失败。**该次尚未验证自主前进/完整 agent 运行：**当时 sim profile 的 `depth_semantics: unverified` 和 `braking_margin_m: 25.0` 使 `forward_max_m=0`；仅证明深度和地面运行接线，不等于“静态场景闭环通过”。已撤除的运动中深度守卫**不再**是能力或验收项；深度断流/新近障不触发自动急停。该阶段没有凭一次 0.196 m 记录静默降低 25 m 停止门槛；其后按用户新决策采用文末最新约定。S7 暂不作验收提交，以免把受控烟测写成完整闭环通过。

本轮没有真机飞行，没有清理其他仓库的 dirty 文件，没有将模型密钥、原始图像或运行资产加入新仓库。

## 2026-09-26 最新执行约定与离线状态

按用户新确认的静态场景方案，仿真 profile 已将 `depth_semantics` 改为 `perspective_ray_m`、`braking_margin_m` 改为 0.60 米；连同测量 0.15 米和延迟 0.25 米，合计计划余量 1 米。`depth_max_m=20` 米，仿真 `max_forward_m=19` 米，原 `max_relative_move_m=0.3` 米继续约束侧向/后退等非纯正向调用。真机 profile、深度无效时拒绝前进及逐动作 HITL 不变。此变更只适用于用户选定的 scene 9、13、21、24；scene 7 旧起点不可拿来证明新规则可飞。

仿真 `forward` 对有效正数的超限提议自动缩短到同号深度规则和 19 米两者的较小值，工具结果记录请求与指令距离、障碍/净空和缩短原因。上限为零或深度未知时返回成功处理的“未移动”结果，但不发布前进 setpoint；runtime 在取得一张新 RGB-D 后继续让同一 VLM 决策。无新 RGB 时仍停止，负数、零、布尔值、NaN/Inf、观测错号仍拒绝。平移无可测位移达到连续 3 次仍停止；成功非零旋转且收到新观测会重置该计数，零度旋转不会。前进运动中不检查新深度，仍保留原 `move` 超时、人工中断和 PX4 悬停/失控交接。

纯离线验证：`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q` **112 passed**，包含 20 米远值、1.5/1.0 米障碍边界、0 米无命令/新观测反馈、自动缩短、仿真正向专属 19 米、真机原硬拒绝与 HITL、旋转重置计数、原 `move` 超时交接。**本轮未启动 AirSim/PX4/ROS、未调用付费 VLM，也没有真实仿真飞行证据。** 1 米为深度计算和目标指令的计划余量，不能视为实测停距或最终物理净空保证；19 米位置 setpoint 的执行时间、超调和完整交互时序由用户后续真实仿真核验。在这些证据返回之前，S7 继续标记未验收，不作通过提交。

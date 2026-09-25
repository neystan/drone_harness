# 真机迁移与现场验收

本文适用于通过 MAVROS 连接 PX4 的真机。它记录当前项目的部署边界和首飞前验收项，不替代 PX4、机架、电池和场地的安全手册。

## 当前项目边界

- `Px4Controller` 通过默认 `/mavros` namespace 读写 MAVROS topic 和 service。
- `drone_agent_real` 不启动 MAVROS、不启动相机驱动，也不启动 AirSim。
- 真机 profile 启用 HITL；起飞前要求已收到 PX4 状态和电池状态，最低电量为 30%。
- 当前真机限值：起飞高度 3 m、水平移动 5 m、垂直移动 2 m、旋转 180°、单动作超时 20 s。
- 程序的状态门控不等于对 RTK 质量、RC 接管、地理围栏和 PX4 failsafe 的完整验证；这些必须在飞控侧单独完成。

## 1. 串口与 MAVROS

先拆桨完成通信验收。使用稳定的 `/dev/serial/by-id/…` 路径，不要把易变化的 `/dev/ttyUSB0` 写入长期启动脚本。

```bash
source /opt/ros/humble/setup.bash
ros2 launch mavros px4.launch \
  fcu_url:="serial:///dev/serial/by-id/<pixhawk-serial-device>:115200" \
  tgt_system:=1 \
  tgt_component:=1
```

若现场暂时只能使用 `/dev/ttyUSB0`，先确认权限和设备归属：

```bash
ls -l /dev/ttyUSB0
groups
```

运行 MAVROS 后，以下检查都必须有数据。其中 `/mavros/state` 必须显示 `connected: true`；电池百分比必须为 0 到 1 的有限数值。

```bash
ros2 topic echo /mavros/state --once
ros2 topic echo /mavros/local_position/pose --once
ros2 topic echo /mavros/extended_state --once
ros2 topic echo /mavros/battery --once
ros2 service list | grep '^/mavros/'
```

## 2. RTK、RC 与 PX4 failsafe

在启动 Agent 前，使用 QGroundControl 和实体遥控器逐项验收：

1. RTK/GNSS 已达到现场允许的定位质量，EKF 估计正常，且 `/mavros/local_position/pose` 持续更新。
2. SBUS 遥控器能解锁、切换模式和人工接管；先验证飞控不依赖 Agent 也能安全降落。
3. 电池、地理围栏、返航点和低电量动作已按现场规则配置并验证。
4. 配置并记录 Offboard-loss 的延时与动作；项目在无法确认悬停时会停止 position-hold，让 PX4 的 Offboard-loss failsafe 接管。
5. 在安全场地、遥控器在手的条件下，先做一次不依赖 Agent 的低空悬停和降落。

不要仅因 ROS2 topic 存在就认为上述飞控级保护已生效。

## 3. RGB 相机

真机不使用 `takeoff_camera.launch.py`，它只服务 AirSim。应先启动深度相机厂商的 ROS2 驱动，并确认 RGB topic：

```bash
ros2 topic list | grep -i image
ros2 topic info <rgb-image-topic>
```

将确认后的 topic 写入 `drone_agent/config/profiles/real.yaml` 的 `ros.camera_scene_topic`；未配置时，`drone_agent_real` 不会订阅相机，飞控控制仍可运行，但拍照和视觉工具不可用。

## 4. 启动顺序

完成飞控与相机验收后，在独立终端依次运行：

```bash
# 终端 1：MAVROS（使用第 1 节的串口命令）

# 终端 2：相机厂商驱动
<camera-driver-launch-command>

# 终端 3：Agent
cd ~/hw-ros2/ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
drone_agent_real
```

`drone_agent_real` 启动后，先执行状态、电池和位置查询。首次受控飞行只允许低高度起飞、短时悬停和降落；确认每一步的 PX4 状态后，再增加移动或视觉任务。

## 5. 首飞前的停止条件

任一项不满足时，不启动飞行动作并排查原因：

- MAVROS 未连接，或状态/位置/电池 topic 缺失、停更或包含无效数值。
- 电量低于 30%，或电池百分比不可用。
- RTK/EKF 未就绪，或本地位置不稳定。
- RC 接管、返航、降落或 Offboard-loss failsafe 未在现场验证。
- RGB topic 未按计划可用时，不执行依赖视觉结果的飞行任务。

# drone_agent

`drone_agent` 是一个使用 ROS2、MAVROS 和 PX4 的自然语言无人机控制 Agent。控制器继续使用原有路径 `drone_agent/px4/controller.py` 和类名 `Px4Controller`，不新增 `mavros/` 目录。

## 安装与构建

假设 ROS2 workspace 为 `~/hw-ros2/ros2`：

```bash
cd ~/hw-ros2/ros2
colcon build --packages-select drone_agent
source install/setup.bash
```

运行前配置模型：

```bash
mkdir -p ~/.config/drone_agent
cp ~/hw-ros2/ros2/src/drone_agent/settings.example.json ~/.config/drone_agent/settings.json
```

然后在 `settings.json` 中填写 LLM/VLM 的 API 配置。

## MAVROS 仿真流程

启动 UE4/AirSim 场景后，在 WSL 中依次执行：

```bash
cd ~/PX4-Autopilot
make px4_sitl_default none_iris
```

另开终端单独启动 MAVROS：

```bash
source /opt/ros/humble/setup.bash
ros2 launch mavros px4.launch \
  fcu_url:="udp://:14540@127.0.0.1:14580" \
  tgt_system:=1 \
  tgt_component:=1
```

再开终端启动 AirSim bridge 和 RGB 相机预览：

```bash
source /opt/ros/humble/setup.bash
source ~/hw-ros2/ros2/install/setup.bash
ros2 launch drone_agent takeoff_camera.launch.py
```

该 launch 只启动 AirSim bridge 和相机预览，并订阅 AirSim RGB topic：
`/airsim_node/PX4/CameraDepth1/Scene`。不启动 MAVROS，也不再使用 `MicroXRCEAgent`。

确认 MAVROS 已连接：

```bash
ros2 topic echo /mavros/state --once
```

最后启动 Agent：

```bash
drone_agent_sim
```

也可以使用 `ros2 run drone_agent drone_agent_sim` 启动。

## 真机迁移

真机通过 MAVROS 独立连接 PX4；`drone_agent_real` 不会启动 MAVROS、相机驱动或 AirSim。
真机接入、相机 topic 配置、RTK/RC/failsafe 验收和首飞步骤见
[`docs/REAL_VEHICLE_MIGRATION.md`](docs/REAL_VEHICLE_MIGRATION.md)。

不要在真机上启动 `takeoff_camera.launch.py`，它仅用于 AirSim 仿真相机链路。

## 目录

- Python 主代码：`drone_agent/`
- ROS2 包文件：`package.xml`、`setup.py`、`launch/`
- 仿真配置：`drone_agent/config/profiles/sim.yaml`
- 相机 topic：`/airsim_node/PX4/CameraDepth1/Scene`

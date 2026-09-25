"""集中定义 PX4 DDS topic 常量。"""

OFFBOARD_CONTROL_MODE_TOPIC = "/fmu/in/offboard_control_mode"
TRAJECTORY_SETPOINT_TOPIC = "/fmu/in/trajectory_setpoint"
VEHICLE_COMMAND_TOPIC = "/fmu/in/vehicle_command"

VEHICLE_LOCAL_POSITION_TOPIC = "/fmu/out/vehicle_local_position"
VEHICLE_STATUS_TOPIC = "/fmu/out/vehicle_status"
BATTERY_STATUS_TOPIC = "/fmu/out/battery_status"
VEHICLE_COMMAND_ACK_TOPIC = "/fmu/out/vehicle_command_ack"

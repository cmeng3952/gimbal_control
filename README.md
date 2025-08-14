# gimbal_control (ROS 2 Humble)

Standalone gimbal control package with UDP control and MQTT command/state bridge. This version targets ROS 2 Humble on Ubuntu 22.04.

## Features
- Control Z2Mini (v2) gimbal over UDP
- MQTT command subscription and state publication
- Keyboard control (optional) for quick testing
- ROS 2 parameters for network and MQTT configuration
- ROS 2 launch file

## Requirements
- Ubuntu 22.04
- ROS 2 Humble
- System dependencies (installed via apt or rosdep):
  - nlohmann-json3-dev
  - libpaho-mqtt-dev
  - libpaho-mqttpp-dev

## Build (colcon)
Recommended workspace layout:
```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/ros2_ws/src
cp -r /home/uav/gimbal_control ~/ros2_ws/src/
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## Run
- Launch (ROS 2):
```bash
ros2 launch gimbal_control gimbal_control.launch.py
```
- Or run the node directly with parameter overrides:
```bash
ros2 run gimbal_control gimbal_control_node --ros-args \
  -p target_ip:=192.168.144.108 -p target_port:=2337 \
  -p local_ip:=192.168.144.111 -p local_port:=2338 \
  -p mqtt_broker:=tcp://<broker-host>:<port> \
  -p mqtt_user:=<user> -p mqtt_password:=<password> \
  -p mqtt_sub_topic:=uavcontrol/gimbal/command/uav1 \
  -p mqtt_pub_topic:=uavcontrol/gimbal/state/uav1
```

## Parameters
- target_ip (string): Gimbal IP (default: 192.168.144.108)
- target_port (int): Gimbal UDP port (default: 2337)
- local_ip (string): Local NIC IP (default: 192.168.144.111)
- local_port (int): Local UDP port (default: 2338)
- mqtt_broker (string): MQTT URL, e.g. tcp://host:port
- mqtt_user (string): MQTT username
- mqtt_password (string): MQTT password
- mqtt_sub_topic (string): Topic for incoming commands
- mqtt_pub_topic (string): Topic for outgoing states/replies

## Keyboard Controls (optional)
With the node focused in a terminal:
- W/S: pitch up/down
- A/D: yaw left/right
- Q/E: roll left/right
- R: center
- + / -: change step sizes
- 0: reset step sizes to defaults
- Enter: toggle smooth mode
- ESC: quit

## MQTT API
- Command subscribe topic: mqtt_sub_topic
- State/response publish topic: mqtt_pub_topic

Commands (JSON):
```json
{"cmd":"center"}
```
```json
{"cmd":"set_angle", "pitch": 10.0, "yaw": -15.0, "roll": 0.0, "smooth": true, "duration": 2.0, "steps": 20}
```
```json
{"cmd":"rotate_rel", "dp": 5.0, "dy": 0.0, "dr": 0.0}
```

State publish (example):
```json
{"angles": {"pitch": 0.0, "yaw": 0.0, "roll": 0.0}}
```

Responses (success flag):
```json
{"ok": true}
```

## Notes
- Ensure your network IPs/ports match the gimbal and local interface.
- MQTT must be reachable from the machine running this node.

## License
MIT

## Maintainer
UAV Team <cmeng3952@gmail.com>

## Push to GitHub
1) Create a new empty repository on GitHub (do not add README/License on GitHub side to avoid history conflicts), e.g. `https://github.com/<your_user>/<your_repo>.git`.
2) In this project directory:
```bash
cd /home/uav/gimbal_control
git init
git add .
git commit -m "ROS 2 Humble migration: ament_cmake, rclcpp, launch, MQTT bridge"
# set default branch to main if needed
git branch -M main
# add your remote
git remote add origin https://github.com/<your_user>/<your_repo>.git
# push
git push -u origin main
```

If the remote already exists or uses SSH, replace the URL accordingly:
```bash
git remote set-url origin git@github.com:<your_user>/<your_repo>.git
```

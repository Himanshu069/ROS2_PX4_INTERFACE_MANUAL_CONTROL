# ROS2_PX4_INTERFACE_MANUAL_CONTROL

**ROS 2–based Manual Teleoperation Interface for PX4 Drones**

This package uses the **ROS 2 PX4 Interface Library** to create a **custom Teleoperation flight mode** in **QGroundControl (QGC)** for manual control of PX4 drones.

It enables **terminal based command input**, direct interaction with PX4 flight modes, and supports both **single drone** and **multi drone** simulation using PX4 SITL.

---
4x sped up video with gazebo simulation running at 10% speed

https://github.com/user-attachments/assets/834726e6-0ccd-477c-8b5f-176bf68d6f15



##  Features

- Custom **Teleoperation Mode** selectable in QGroundControl  
- Direct PX4 control via **ROS 2 PX4 Interface Library**
- **Keyboard-based teleoperation** from terminal
- Works with **PX4 SITL + Gazebo (gz)**
- Supports **multi-vehicle simulation**

---

## Version Compatibility (IMPORTANT)

> **PX4, `px4_msgs`, and `px4-ros2-interface-library` versions MUST match**

This repository is tested with:

| Component | Version |
|---------|--------|
| PX4 Autopilot | `release/1.16` |
| px4_msgs | `release/1.16` |
| px4-ros2-interface-library | `release/1.16` |

---

## Prerequisites

- ROS 2 
- PX4 Autopilot
- Gazebo (gz)
- QGroundControl


---

##  Workspace Setup

```bash
mkdir -p ~/drone_ws/src
cd ~/drone_ws/src
```

---

## Clone Dependencies

```bash
git clone -b release/1.16 https://github.com/PX4/px4_msgs.git
git clone -b release/1.16 https://github.com/Auterion/px4-ros2-interface-lib.git
git clone https://github.com/Himanshu069/ROS2_PX4_INTERFACE_MANUAL_CONTROL
```

---

## Build

```bash
cd ~/drone_ws
colcon build --packages-select px4_msgs px4_ros2_cpp
source install/setup.bash
colcon build
```

---

## Run PX4 SITL
In a new terminal, run
```bash
cd ~/PX4_Autopilot
make px4_sitl gz_x500_baylands
```

---

## QGroundControl
In a separate terminal,
```bash
./QGroundControl-x86_64.AppImage
```
Also, start uXRCE-DDS client in a new terminal
```bash
MicroXRCEAgent udp4 -p 8888
```
---

## Teleoperation
IN one terminal, run
```bash
ros2 run manual_ros2_px4_multi teleop
```
And , in another terminal,run
```bash
ros2 run manual_ros2_px4_multi teleop_keyboard
```
<img width="534" height="266" alt="Screenshot from 2026-01-17 19-43-02" src="https://github.com/user-attachments/assets/97a9c2d8-063d-439e-90f8-21979cc1bbc6" />
<br>
You can control the drone by giving teleop commands in this terminal.
<br>

---
You need to first takeoff the drone then select Teleoperation mode once the drone is in Hover mode in QGC , then you can send commands from the terminal to control the drone

<img width="644" height="414" alt="Screenshot from 2026-01-17 19-40-47" src="https://github.com/user-attachments/assets/2a81bf17-f60e-4ec1-9f2c-23d88d7b4121" />

---

## For Multi Drone
Firstly , spawn multiple drones as per PX4 docs https://docs.px4.io/main/en/sim_gazebo_gz/multi_vehicle_simulation
```bash
ros2 run manual_ros2_px4_multi teleop_keyboard -p instance:=px4_1
```
And , in another terminal,
```bash
ros2 run manual_ros2_px4_multi teleop -p ns__:=px4_1

```
---

##  Author

**Himanshu Paudel**  
GitHub: https://github.com/Himanshu069

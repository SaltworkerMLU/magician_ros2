# A Dobot Magician control stack using ROS2 Humble
<img src="https://img.shields.io/badge/ros--version-humble-blue"/>  <img src="https://img.shields.io/badge/platform%20-Ubuntu%2022.04-orange"/>  [![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

<p align="center">

<img src=.github/images/dobot_magician_gripper.webp width=250 height=250><img src=".github/images/dobot_menu_control.png" width="200" height="250"/><img src=".github/images/HumbleHawksbill.webp" width="200" height="250"/>
</p> 

**This project was originally forked from this GitHub-repository: https://github.com/jkaniuka/magician_ros2**

## Table of contents :clipboard:
* [Installation](#installation)
* [How to start up the code](#startup)
* [Dobot Menu](#dobot_menu)
    * [Control Panel](#control_panel)
    * [Speed Panel](#speed_panel)
    * [Tool Panel](#tool_panel)
    * [Scripting Panel](#scripting_panel)
    * [Info Panel](#info_panel)
    * [Logger Panel](#logger_panel)
* [A list of ROS2 commands](#commands)
* [DOCUMENTATION.md](#documentation)

<a name="installation"></a>
## Installation :arrow_down:
[!NOTE] 
This repository uses ~200MB (~5GB including ROS2 Humble and other installed packages).

This section provides a short guide to installing this repository using ROS2 Humble. If you want to try installing this repository not using Ubuntu-22.04 + ROS2 Humble, do so at your own discretion.


ROS2 compatibility | Compatible
--- | --- |
Ubuntu-25.04 + ROS2 Kilted | ❓
Ubuntu-24.04 + ROS2 Jazzy | ❌
Ubuntu-23.04 + ROS2 Iron | ❓
Ubuntu-22.04 + ROS2 Humble | ✅
Ubuntu-20.04 + ROS2 Galactic | ❓
...  | ❓

First, install Ubuntu-22.04 using one of the below options:

- Install WSL2 by opening **command prompt** and typing `wsl install -d Ubuntu-22.04`. *Only works with windows*
- Use a virtual machine (VM). If you're new to VMs, I recommend VMware.

[!IMPORTANT] 
In case you are using WSL2, you need to [install usbipd](https://github.com/dorssel/usbipd-win/releases/tag/v5.3.0) for windows to be able to transfer USB data to WSL2. In case you are using a VM, allow data transfer using USB ports with the command* `sudo usermod -a -G dialout <username>`.

<a name="installation_repo"></a>
### How to install this repository

Start off by opening a **linux terminal**. If you are using WSL2, do so by opening **command prompt** and typing `wsl -d Ubuntu-22.04`

Inside the opened **linux terminal**, paste the following:

```
cd ~
mkdir -p ~/ws_magician/src # Creates directory
git clone https://github.com/SaltworkerMLU/magician_ros2.git ~/ws_magician/src --branch magician_ros2_MLU
bash ~/ws_magician/src/terminal/install_dependencies.bash
```

When this is done, reboot linux. If you are using WSL2, open a new **command prompt** and type `wsl --shutdown` followed by `wsl -d Ubuntu-22.04`

After rebooting linux, copy the following into your terminal: 

```
cd ~/ws_magician
colcon build
```
[!WARNING] 
Building using **colcon build** from the wrong folder will cause the build to not load when prompted.

[!TIP] 
If you are using WSL2, open a **command prompt** and run the command* `cp ~/ws_magician/src/terminal/dobot_bootup.bat /mnt/c/users/YOUR_USERNAME/Desktop` to copy the windows batch file used to connect the dobot magician via usb to WSL2 into the desktop folder.

<a name="startup"></a>
## How to start up the code :arrow_up:

[!IMPORTANT] 
If you are using WSL2, run the windows batch file **dobot_bootup.bat** run the code

[!IMPORTANT]
IF you are using VM, open a **linux terminal** and run the command `bash ~/ws_magician/src/terminal/dobot_bootup.bash`.

[!TIP]
If you want to edit the source code, simply open a linux terminal, navigate to ~/ws_magician, and then type `code .`. This will open Visual Studio Code and install it if not done. After editing the code, use `colcon build` to build your revamped version of the src.

<a name="dobot_menu"></a>
## Dobot Menu :bar_chart:
The Dobot Menu is the window that pops up when all ROS2 nodes have been initialized. It consists of the following panels:
* Control Panel
* Speed Panel
* Tool Panel
* scripting Panel
* Info Panel
* Logger Panel

<a name="control_panel"></a>
### Control Panel
<img align="left" src=".github/images/dobot_menu_control.png" width="250" height="300"/>

The control panel allows the user to control the Dobot Magician joint by joint or axis by axis in terms of the base frame. 

The red button is an "emergency stop button" made to stop any external scripts from being run on the Dobot Magician.

The home icon is the "homing button" made to home the dobot into its original position. This is done specifically to calibrate the sensors (the encoders). The Dobot Magician is homed automatically after opening the Dobot Menu.

<br clear="left"/>

<a name="speed_panel"></a>
### Speed Panel

<img align="left" src=".github/images/dobot_menu_speed.png" width="250" height="300"/>
The speed panel allows the user to adjust the joint and TCP velocities and acceleration. 

The adjustments are applied to the Control Panel only, meaning the velocity and acceleration of commands from the scripting panel must be set there.

<br clear="left"/>

<a name="tool_panel"></a>
### Tool Panel
<img align="left" src=".github/images/dobot_menu_tool.png" width="250" height="300"/>

The tool panel allows the user to use the end-effectors which have a separate function. This includes the Gripper and Suction Cup which come with the Dobot Magician kit.

<br clear="left"/>

<a name="scripting_panel"></a>
### Scripting Panel
<img align="left" src=".github/images/dobot_menu_scripting.png" width="250" height="300"/>

The scripting panel allows the user to create their own scripts to run on the dobot magician by using the "Execute" button. Separate scripts can be made by writing a file name. These scripts can be exported to an export path by using the "Export Program" button and imported by writing the import folder and file name. Alternatively, you can find the command files inside the folder

`cd ~/ws_magician/src/install/dobot_menu/share/dobot_menu/resource/commands`

[!NOTE] 
Using other commands than the ones inside the scripting Panel will result in the Dobot Menu crashing.

[!TIP] 
You can write inside section where the code lies.

<br clear="left"/>

<a name="logger_panel"></a>
### Logger Panel
<img align="left" src=".github/images/dobot_menu_logger.png" width="250" height="300"/>
The logger panel is made for the user to both make personal notes and to log e.g. error codes, trajectory points, etc.

Similarly with the scripting Panel, the user can change log file by entering a new log file name. These logs are stored in the folder 

`cd ~/ws_magician/src/install/dobot_menu/share/dobot_menu/resource/logs`

<br clear="left"/>

<a name="commands"></a>
## A list of ROS2 commands :clipboard:
[!IMPORTANT]
Remember to run the command `bash ~/ws_magician/src/terminal/dobot_bootup.bash` and letting it complete initialization before proceeding with the following commands. Otherwise, the commands will not work.

Example script | description
--- | --- |
`ros2 run dobot_demos test_gripper` | Tests gripper
`ros2 run dobot_demos test_suction_cup` | Tests suction cup
`ros2 run dobot_demos test_homing` | Homes the dobot
`ros2 run dobot_demos test_point_to_point` | Moves to a certain spot using PTP movement
`ros2 run dobot_demos test_pick_and_place` | Does a sequence of PTP movements while picking and placing objects in the dobot's way.

Service command | description
--- | --- |
`ros2 service call /dobot_homing_service dobot_msgs/srv/ExecuteHomingProcedure` | Initiates homing of the dobot.
`ros2 service call /dobot_gripper_service dobot_msgs/srv/GripperControl "{gripper_state: 'open', keep_compressor_running: true}"` | `gripper state` (type _string_) : _open/close_ & `keep_compressor_running` (type _bool_) : _true/false_
`ros2 service call /dobot_suction_cup_service dobot_msgs/srv/SuctionCupControl "{enable_suction: true}"` | `enable_suction` (type _bool_) : _true/false_

Action command | description
--- | --- |
`ros2 action send_goal /PTP_action  dobot_msgs/action/PointToPoint "{motion_type: 1, target_pose: [150.0, 0.0, 100.0, 0.0], velocity_ratio: 0.5, acceleration_ratio: 0.3}" --feedback` | Trajects the dobot towards the specified target_pose and motion_type. Refer to [DOCUMENTATION.md](https://github.com/SaltworkerMLU/magician_ros2/blob/main/DOCUMETNATION.md) to learn more.
`ros2 action send_goal /Arc_action  dobot_msgs/action/ArcMotion "{pc: [175.0, 25.0, 100.0, 0.0], pf: [200.0, 0.0, 100.0, 0.0], velocity_ratio: 0.5, acceleration_ratio: 0.3}" --feedback` | Trajects the dobot toward the specified ending_point via. the circumference_point, resulting in a curved arc-like movement. Refer to [DOCUMENTATION.md](https://github.com/SaltworkerMLU/magician_ros2/blob/main/DOCUMETNATION.md) to learn more.
`ros2 action send_goal /draw_polygon dobot_msgs/action/DrawPolygon "{target_pose: [150, 0, 100, 0], radius: 25, z_level: -74, sides: 3, theta: 0, velocity_ratio: 0.5, acceleration_ratio: 0.3}" --feedback` | Draws a horizontal circle. Use this command with the pen gripper. Refer to [DOCUMENTATION.md](https://github.com/SaltworkerMLU/magician_ros2/blob/main/DOCUMETNATION.md)  to learn more.

[!TIP]
Adding `--feedback` flag will cause the terminal to display the current position of the robot while it is moving

Other commands | image
--- | --- |
`rqt -s rqt_robot_monitor` | <img src="https://user-images.githubusercontent.com/80155305/220293202-d1320648-719d-4e3c-a592-52e8607d3838.png" width="240" height="250"/> <img src="https://user-images.githubusercontent.com/80155305/220293214-8e07c4ef-67fa-40c1-a562-7c97e81730ff.png" width="166" height="250"/>
`rqt -s dobot_menu` | <img src=".github/images/dobot_menu_control.png" width="350" height="400"/>
`ros2 launch` <br> `dobot_description` <br> `display.launch.py DOF:=3 `<br>` tool:=none `<br>` use_camera:=false` | <img src=".github/images/dobot_simulator_3D.png" width=240 height=250>


<a name="documentation"></a>
## DOCUMENTATION.md :scroll:

To those who are interested in learning more about how the src is configured, the file [DOCUMENTATION.md](
https://github.com/SaltworkerMLU/magician_ros2/blob/main/DOCUMETNATION.md) goes into detail on topics like:
* Dobot Magician Workspace
* Dobot Magician Kinematics
* Trajectory validation
usbipd bind --hardware-id 10c4:ea60
usbipd attach --wsl Ubuntu-22.04 --hardware-id 10c4:ea60
wsl bash -c "source /opt/ros/humble/setup.bash; source ~/ws_magician/install/setup.bash; export MAGICIAN_TOOL=none; ros2 launch dobot_bootup dobot_magician_control_system.launch.py"
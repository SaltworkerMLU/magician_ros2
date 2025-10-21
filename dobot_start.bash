gnome-terminal -- sh -c "bash -c \"ros2 launch dobot_bootup dobot_magician_control_system.launch.py; exec bash\"";
#gnome-terminal -- sh -c "bash -c \"ros2 launch dobot_description display.launch.py DOF:=3 tool:=none use_camera:=false; exec bash\"";
gnome-terminal -- sh -c "bash -c \"rqt --force-discover --clear-config -s dobot_menu; exec bash\"";
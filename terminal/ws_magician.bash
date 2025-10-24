project_name=$1

sudo apt install ros-humble-diagnostic-aggregator ros-humble-rqt-robot-monitor python3-pykdl;
sudo apt install gnome-terminal;

source /opt/ros/humble/setup.bash;
mkdir -p ~/ws_magician/src;
git clone https://github.com/jkaniuka/magician_ros2.git ~/ws_magician/src; 
cd ws_magician;

sudo rosdep init;
rosdep update;

rosdep install -i --from-path src --rosdistro humble -y;
colcon build;

sudo usermod -a -G dialout vboxuser;

echo "" >> ~/.bashrc; # Creates empty line
echo "export MAGICIAN_TOOL=none" >> ~/.bashrc;
echo "source ~/ws_magician/install/setup.bash" >> ~/.bashrc;

exit;
#kill -9 $PPID;
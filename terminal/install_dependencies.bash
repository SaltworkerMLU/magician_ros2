bash ~/ws_magician/src/terminal/ros2humble.bash

sudo apt install ros-humble-tf-transformations

# ---------------------------- #
# Repo dependencies start here #
# ---------------------------- #

# Install usbipd dependencies
sudo apt install linux-tools-virtual hwdata
sudo update-alternatives --install /usr/local/bin/usbip usbip `ls /usr/lib/linux-tools/*/usbip | tail -n1` 20
sudo modprobe vhci_hcd

sudo apt install pip
sudo apt-get install python3-pip

sudo apt install ros-humble-diagnostic-aggregator ros-humble-rqt-robot-monitor python3-pykdl;
sudo apt install gnome-terminal;
sudo apt install gedit; # Not necessary but useful

sudo rosdep init;
rosdep update;

rosdep install -i --from-path src --rosdistro humble -y;

pip install -r ~/ws_magician/src/requirements.txt

echo "" >> ~/.bashrc; # Creates empty line
echo "export MAGICIAN_TOOL=none" >> ~/.bashrc;
echo "source ~/ws_magician/install/setup.bash" >> ~/.bashrc;
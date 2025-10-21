# Use sudo to print password <... sudo -S echo "changeme">. 
# This prompts password which is entered automatically using <echo "changeme" | ...>
echo "changeme" | sudo -S echo "changeme";

# Access /etc/sudoers using sudo (after successful password prompt) to make sudo not prompt password.
echo "vboxuser ALL=(ALL) NOPASSWD: ALL" | sudo tee -a /etc/sudoers;

sudo apt install gedit; # Install GUI text editor - Not necessary for ROS2

# Install usbipd dependencies
sudo apt install linux-tools-virtual hwdata
sudo update-alternatives --install /usr/local/bin/usbip usbip `ls /usr/lib/linux-tools/*/usbip | tail -n1` 20
sudo modprobe vhci_hcd

sudo apt-get install python3-pip

############################################
### ROS2 Humble Installation starts here ###
############################################

locale;  # check for UTF-8

sudo apt update && sudo apt install locales;
sudo locale-gen en_US en_US.UTF-8;
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8;
export LANG=en_US.UTF-8;

locale;  # verify settings

sudo apt install software-properties-common;
sudo add-apt-repository universe;

sudo apt update && sudo apt install curl -y;
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}');
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb";
sudo dpkg -i /tmp/ros2-apt-source.deb;

sudo apt update;

sudo apt upgrade;

sudo apt install ros-humble-desktop-full;

sudo apt install ros-dev-tools;

echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc;

exit;
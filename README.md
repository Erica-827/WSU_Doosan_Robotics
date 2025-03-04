# WSU_Doosan_Robotics
Welcome to the working repository for the WSU Robots assisting patient mobility project. Cloning this repo will give you all the working packages other than the Doosan Robotics package.<br> <br>
The Doosan Robotics Package can be cloned from https://github.com/doosan-robotics/doosan-robot

# ROS Installation and Setup

1. Install a VM of Ubuntu 22.04

2. Installation of packages and dependencies

ROS 2 Humble Install
```
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
# Add ROS 2 GPG key
sudo curl - sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key-o/usr/share/keyrings/ros-archive-keyring.gpg1
# Add ROS 2 repository to sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr\/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2\/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
# Update package lists
sudo apt update
sudo apt upgrade
```
Installing Dependencies
```
# Update and install general dependencies
sudo apt-get update
sudo apt-get install -y libpoco-dev libyaml-cpp-dev wget
Install ROS 2 specific packages
sudo apt-get install -y ros-humble-control-msgs ros-humble-realtime-tools ros-humble-xacro ros-humble-joint-state-publisher-gui ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-gazebo-msgs ros-humble-moveit-msgsdbus-x11 ros-humble-moveit-configs-utils ros-humble-moveit-ros-move-group
# Install ROS 2 desktop ( includes visualization tools andsimulators )
sudo apt install ros-humble-desktop
# Install ROS 2 development tools
sudo apt install ros-dev-tools
```
ROS 2 Setup
```
# Source the ROS 2 setup file
source /opt/ros/humble/setup.bash
Add to . bashrc to automatically source on terminal startup
echo ’ source/opt/ros/humble/setup.bash ’ >> ~/.bashrc
```
Docker Installation
```
# Add Docker ’ s official GPG key
sudo apt-get update
sudo apt-get install ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl - fsSL https ://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc
# Add the repository to Apt sources
echo \ "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update
```
Intall Docker Packages
```
sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```
Gazebo Installation
```
# Add Gazebo repository
sudo sh -c ’ echo " deb http :// packages . osrfoundation . org / gazebo /ubuntu - stable ‘ lsb_release -cs ‘ main " > / etc / apt / sources . list . d/ gazebo - stable . list ’
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt- key add -
sudo apt-get update
# Install Gazebo and related ROS 2 packages
sudo apt-get install -y libignition-gazebo6-dev
sudo apt-get install -y ros-humble-gazebo-ros-pkgs ros-humble-moveit-msgs ros-humble-ros-gz-sim ros-humble-ros-gz
```
3. Setup the ROS workplace <br>

Clone the Github Repo
```
git clone https://github.com/Erica-827/WSU_Doosan_Robotics.git
cd WSU_Doosan_Robotics/src
git clone https://github.com/doosan-robotics/doosan-robot.git
cd ..
```
Build the files, this may take a while
```
colcon build
```

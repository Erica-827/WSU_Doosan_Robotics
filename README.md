# WSU_Doosan_Robotics
Welcome to the working repository for the WSU Robots assisting pacient mobility project. Cloning this repo will give you all the working packages other than the Doosan Robotics package.<br> <br>
The Doosan Robotics Package can be cloned from https://github.com/doosan-robotics/doosan-robot

# To setup ROS follow the following steps

1. Install a VM of Ubuntu 22.04

2. Run the following Commands
```
sudo apt install software - properties - common
sudo add - apt - repository universe
sudo apt update && sudo apt install curl -y
# Add ROS 2 GPG key
sudo curl - sSL https :// raw . githubusercontent . com / ros /rosdistro / master / ros . key -o / usr / share / keyrings / ros -archive - keyring . gpg1
# Add ROS 2 repository to sources list
echo " deb [ arch = $ ( dpkg -- print - architecture ) signed - by =/ usr /share / keyrings / ros - archive - keyring . gpg ] http :// packages .ros . org / ros2 / ubuntu $ (. / etc / os - release && echo$UBUNTU_CODENAME ) main " | sudo tee / etc / apt / sources . list . d/ ros2 . list > / dev / null
# Update package lists
sudo apt update
sudo apt upgrade
```

# setup ros2

## Source: https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros_interface/ros2/software_setup.html

sudo apt update

sudo apt install curl vim git -y

curl 'https://raw.githubusercontent.com/Interbotix/interbotix_ros_manipulators/humble/interbotix_ros_xsarms/install/amd64/xsarm_amd64_install.sh' > xsarm_amd64_install.sh

source /opt/ros/$ROS_DISTRO/setup.bash

chmod +x xsarm_amd64_install.sh

./xsarm_amd64_install.sh -d humble -n

source /opt/ros/$ROS_DISTRO/setup.bash

source ~/interbotix_ws/install/setup.bash

ros2 pkg list | grep interbotix


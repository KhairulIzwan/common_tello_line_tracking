# Common Tello Line-Tracking

## Preparation:

1. Install Ubuntu 16.04: 
	Download Source: http://old-releases.ubuntu.com/releases/16.04.4/
	Installation: https://ubuntu.com/tutorials/install-ubuntu-desktop-1604#1-overview
2. Instal ROS Kinetic:
	$ sudo apt-get update
	$ sudo apt-get upgrade
	$ wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_kinetic.sh
	$ chmod 755 ./install_ros_kinetic.sh 
	$ bash ./install_ros_kinetic.sh 
	$ sudo apt-get install ros-kinetic-joy ros-kinetic-teleop-twist-joy ros-kinetic-teleop-twist-keyboard ros-kinetic-laser-proc ros-kinetic-rgbd-launch ros-kinetic-depthimage-to-laserscan ros-kinetic-rosserial-arduino ros-kinetic-rosserial-python ros-kinetic-rosserial-server ros-kinetic-rosserial-client ros-kinetic-rosserial-msgs ros-kinetic-amcl ros-kinetic-map-server ros-kinetic-move-base ros-kinetic-urdf ros-kinetic-xacro ros-kinetic-compressed-image-transport ros-kinetic-rqt* ros-kinetic-gmapping ros-kinetic-navigation ros-kinetic-interactive-markers
	$ sudo apt-get install ros-kinetic-dynamixel-sdk
	$ sudo apt-get install ros-kinetic-turtlebot3-msgs
	$ sudo apt-get install ros-kinetic-turtlebot3
3. Install Tello Driver:
	$ cd ~/catkin_ws/src
	$ git clone https://github.com/KhairulIzwan/tello_driver.git
	$ cd ~/catkin_ws && rosdep install -y --from-paths src --ignore-src --rosdistro kinetic && catkin_make && rospack profile
	$ source ~/.bashrc
	$ rosed tello_driver tello_node.launch
	Edit:
		<node pkg="image_transport" 
			name="image_compressed"
			type="republish"
			args="h264 in:=image_raw compressed out:=image_raw" />
	$ cd ~/catkin_ws && rosdep install -y --from-paths src --ignore-src --rosdistro kinetic && catkin_make && rospack profile
	$ source ~/.bashrc

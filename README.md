# Common Tello Line-Tracking

## Preparation:

1. Install Ubuntu 16.04: 
	1. Download Source: http://old-releases.ubuntu.com/releases/16.04.4/
	2. Installation: https://ubuntu.com/tutorials/install-ubuntu-desktop-1604#1-overview

2. Instal ROS Kinetic:
	```
	$ sudo apt-get update
	$ sudo apt-get upgrade
	$ wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_kinetic.sh
	$ chmod 755 ./install_ros_kinetic.sh 
	$ bash ./install_ros_kinetic.sh 
	$ sudo apt-get install ros-kinetic-joy ros-kinetic-teleop-twist-joy ros-kinetic-teleop-twist-keyboard ros-kinetic-laser-proc ros-kinetic-rgbd-launch ros-kinetic-depthimage-to-laserscan ros-kinetic-rosserial-arduino ros-kinetic-rosserial-python ros-kinetic-rosserial-server ros-kinetic-rosserial-client ros-kinetic-rosserial-msgs ros-kinetic-amcl ros-kinetic-map-server ros-kinetic-move-base ros-kinetic-urdf ros-kinetic-xacro ros-kinetic-compressed-image-transport ros-kinetic-rqt* ros-kinetic-gmapping ros-kinetic-navigation ros-kinetic-interactive-markers
	$ sudo apt-get install ros-kinetic-dynamixel-sdk
	$ sudo apt-get install ros-kinetic-turtlebot3-msgs
	$ sudo apt-get install ros-kinetic-turtlebot3
	```
	
3. Install Tello Driver:
	```
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
	```
	
4. Intall PIP library:
	```
	$ cd ~
	$ wget https://bootstrap.pypa.io/get-pip.py
	$ sudo python get-pip.py
	```
	
5. Install imutils library:
	```
	$ python -m pip install imutils
	```
	
6. Install AprilTag3 library:
	```
	$ python -m pip install apriltag
	```
	
## Operation

1. Manually Control (Using Keyboard):
	1. Activate Tello Driver node:
		1. Terminal 1: 
			```$ roslaunch tello_driver tello_node.launch```
	2. Run Tele-Operation Keyboard node to Control Drone:
		1. Terminal 2: $ rosrun common_tello_line_tracking tello_teleop_key.py
	
2. Camera Preview:
	1. Activate Tello Driver:
		1. Terminal 1: $ roslaunch tello_driver tello_node.launch
	2. Run Camera Preview node:
		1. Terminal 2: $ rosrun common_tello_line_tracking tello_camera_preview.py
	3. Run Tele-Operation Keyboard node to Control Drone:
		1. Terminal 2: $ rosrun common_tello_line_tracking tello_teleop_key.py
	
3. AprilTag3 Detection:
	1. ACtivate Tello Driver node:
		1. Terminal 1: $ roslaunch tello_driver tello_node.launch
	2. Run Camera Preview with AprilTag3 Detection node:
		1. Terminal 2: $ rosrun common_tello_line_tracking tello_camera_apriltag_detection.py
	
4. Autonomous Takeoff and Land (Using AprilTag3 0:Takeoff; 1:Land):
	1. ACtivate Tello Driver node:
		1. Terminal 1: $ roslaunch tello_driver tello_node.launch
	2. Run Camera Preview with AprilTag3 Detection node:
		1. Terminal 2: $ rosrun common_tello_line_tracking tello_camera_apriltag_detection.py
	3. Run Autonomous Takeoff and Land node:
		1. Terminal 3: $ rosrun common_tello_line_tracking tello_camera_apriltag_takeoff_land.py
	
5. Autonomou Task Basic:
	1. ACtivate Tello Driver node:
		1. Terminal 1: $ roslaunch tello_driver tello_node.launch
	2. Run Camera Preview with AprilTag3 Detection node:
		1. Terminal 2: $ rosrun common_tello_line_tracking tello_camera_apriltag_detection.py
	3. 

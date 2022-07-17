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
	
## Nodes
### Tello Driver Node

1. Subscribed topics
```
/tello/cmd_vel geometry_msgs/Twist
/tello/emergency std_msgs/Empty
/tello/fast_mode std_msgs/Empty
/tello/flattrim std_msgs/Empty
/tello/flip std_msgs/Uint8
/tello/land std_msgs/Empty
/tello/palm_land std_msgs/Empty
/tello/takeoff std_msgs/Empty
/tello/manual_takeoff std_msgs/Empty
/tello/throw_takeoff std_msgs/Empty
```

2. Published topics
```
/tello/camera/camera_info sensor_msgs/CameraInfo
/tello/image_raw sensor_msgs/Image
/tello/imag/raw/h264 h264_image_transport/H264Packet
/tello/odom nav_msgs/Odometry
/tello/imu sensor_msgs/Imu
/tello/status tello_driver/TelloStatus
/tello/image_raw/compressed sensor_msgs/CompressedImage
```

3. Services
```
TODO
```

4. Parameters
```
~/tello_driver_node/connect_timeout_sec
~/tello_driver_node/fixed_video_rate
~/tello_driver_node/local_cmd_client_port
~/tello_driver_node/local_vid_server_port
~/tello_driver_node/stream_h264_video
~/tello_driver_node/tello_cmd_server_port
~/tello_driver_node/tello_ip
~/tello_driver_node/vel_cmd_scale
~/tello_driver_node/video_req_sps_hz
~/tello_driver_node/altitude_limit
~/tello_driver_node/attitude_limit
~/tello_driver_node/low_bat_threshold
```

### Tello Teleop Node

1. Subscribed topics
```
TODO
```

2. Published topics
```
/tello/cmd_vel geometry_msgs/Twist
/tello/land std_msgs/Empty
/tello/takeoff std_msgs/Empty
```

3. Services
```
TODO
```

4. Parameters
```
TODO
```

### Tello Camera Preview Node

1. Subscribed topics
```
/tello/image_raw/compressed sensor_msgs/CompressedImage

*used for display purpose only*
/tello/odom nav_msgs/Odometry
/tello/imu sensor_msgs/Imu
/tello/status tello_driver/TelloStatus
```

2. Published topics
```
TODO
```

3. Services
```
TODO
```

4. Parameters
```
TODO
```

### Tello Camera AprilTag3 Detection Node

1. Subscribed topics
```
/tello/image_raw/compressed sensor_msgs/CompressedImage
```

2. Published topics
```
/isApriltag
/isApriltag/Center/X
/isApriltag/Center/Y
/isApriltag/Corner/X1
/isApriltag/Corner/X2
/isApriltag/Corner/X3
/isApriltag/Corner/X4
/isApriltag/Corner/Y1
/isApriltag/Corner/Y2
/isApriltag/Corner/Y3
/isApriltag/Corner/Y4
/isApriltag/Distance
/isApriltag/Homography/H00
/isApriltag/Homography/H01
/isApriltag/Homography/H02
/isApriltag/Homography/H10
/isApriltag/Homography/H11
/isApriltag/Homography/H12
/isApriltag/Homography/H20
/isApriltag/Homography/H21
/isApriltag/Homography/H22
/isApriltag/N
```

3. Services
```
TODO
```

4. Parameters
```
TODO
```
## Operation

1. Manually Control (Using Keyboard):
	1. Activate Tello Driver node:
		1. Terminal 1:
		```$ roslaunch tello_driver tello_node.launch```
	
	2. Run Tele-Operation Keyboard node to Control Drone:
		1. Terminal 2:
		```$ rosrun common_tello_line_tracking tello_teleop_key.py```
	
2. Camera Preview:
	1. Activate Tello Driver:
		1. Terminal 1:
		```$ roslaunch tello_driver tello_node.launch```
		
	2. Run Camera Preview node:
		1. Terminal 2:
		```$ rosrun common_tello_line_tracking tello_camera_preview.py```
		
	3. Run Tele-Operation Keyboard node to Control Drone:
		1. Terminal 3:
		```$ rosrun common_tello_line_tracking tello_teleop_key.py```
	
3. AprilTag3 Detection:
	1. Activate Tello Driver node:
		1. Terminal 1:
		```$ roslaunch tello_driver tello_node.launch```
		
	2. Run Camera Preview with AprilTag3 Detection node:
		1. Terminal 2:
		```$ rosrun common_tello_line_tracking tello_camera_apriltag_detection.py```
	
4. Autonomous Takeoff and Land (Using AprilTag3 0:Takeoff; 1:Land):
	1. Activate Tello Driver node:
		1. Terminal 1:
		```$ roslaunch tello_driver tello_node.launch```
		
	2. Run Camera Preview with AprilTag3 Detection node:
		1. Terminal 2:
		```$ rosrun common_tello_line_tracking tello_camera_apriltag_detection.py```
		
	3. Run Autonomous Takeoff and Land node:
		1. Terminal 3:
		```$ rosrun common_tello_line_tracking tello_camera_apriltag_takeoff_land.py```
	
5. Autonomou Task Basic:
	1. Activate Tello Driver node:
		1. Terminal 1:
		```$ roslaunch tello_driver tello_node.launch```
	
	2. Run	
	3. Run Camera Preview with AprilTag3 Detection node:
		1. Terminal 2:
		```$ rosrun common_tello_line_tracking tello_camera_apriltag_detection.py```
		
	3. 

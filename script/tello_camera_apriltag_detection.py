#!/usr/bin/env python

################################################################################
## {Description}: Detecting an Apriltag3
## {Description}: Publish /isApriltag topic
## {Description}: If AprilTag3 detected; /isApriltag --> True
## {Description}: If AprilTag3 detected; /isApriltag --> False
################################################################################
## Author: Khairul Izwan Bin Kamsani
## Version: {1}.{0}.{0}
## Email: {wansnap@gmail.com}
################################################################################

"""
Image published (CompressedImage) from tello originally size of 960x720 pixels
We will try to resize it using imutils.resize (with aspect ratio) to width = 320
and then republish it as Image
"""

# import the necessary Python packages
from __future__ import print_function
import sys
import cv2
import time
import numpy as np
import imutils
import random
import apriltag

# import the necessary ROS packages
from std_msgs.msg import String, Bool, Float32
from sensor_msgs.msg import Image, CompressedImage

from cv_bridge import CvBridge
from cv_bridge import CvBridgeError

from common_tello_line_tracking.msg import apriltagN as apriltagList
from common_tello_line_tracking.msg import apriltagC as apriltagCenter
from common_tello_line_tracking.msg import apriltagH as apriltagHomography
from common_tello_line_tracking.msg import apriltagCorner
from common_tello_line_tracking.msg import apriltagDistance

import rospy

class CameraAprilTag:
	def __init__(self):

		# OpenCV -- ROS
		self.bridge = CvBridge()
		
		# AprilTag3 
		self.detector = apriltag.Detector()
		
		self.isApriltag = Bool()
		self.isApriltagN = apriltagList()
		self.apriltagCenterX = apriltagCenter()
		self.apriltagCenterY = apriltagCenter()
		self.apriltagH00 = apriltagHomography()
		self.apriltagH01 = apriltagHomography()
		self.apriltagH02 = apriltagHomography()
		self.apriltagH10 = apriltagHomography()
		self.apriltagH11 = apriltagHomography()
		self.apriltagH12 = apriltagHomography()
		self.apriltagH20 = apriltagHomography()
		self.apriltagH21 = apriltagHomography()
		self.apriltagH22 = apriltagHomography()
		self.apriltagCornerX1 = apriltagCorner()
		self.apriltagCornerY1 = apriltagCorner()
		self.apriltagCornerX2 = apriltagCorner()
		self.apriltagCornerY2 = apriltagCorner()
		self.apriltagCornerX3 = apriltagCorner()
		self.apriltagCornerY3 = apriltagCorner()
		self.apriltagCornerX4 = apriltagCorner()
		self.apriltagCornerY4 = apriltagCorner()
		self.apriltagDistance = apriltagDistance()
		
		# state
		self.image_received = False
		
		self.knownWidth = 0.135 # m
		self.perWidth = 120 # pixels
		self.knownDistance = 1 # m
		self.focalLength = (self.perWidth * self.knownDistance) / self.knownWidth
		
		rospy.logwarn("AprilTag Detection Node [ONLINE]...")
		
		# rospy shutdown
		rospy.on_shutdown(self.cbShutdown)
		
		# Subscribe to CompressedImage msg
		self.telloImage_topic = "/tello/image_raw/compressed"
		self.telloImage_sub = rospy.Subscriber(
						self.telloImage_topic, 
						CompressedImage, 
						self.cbImage
						)
						
		# Publish to Bool msg
		self.isApriltag_topic = "/isApriltag"
		self.isApriltag_pub = rospy.Publisher(
					self.isApriltag_topic, 
					Bool, 
					queue_size=10
					)
					
		# Publish to apriltagList msg
		self.isApriltagN_topic = "/isApriltag/N"
		self.isApriltagN_pub = rospy.Publisher(
					self.isApriltagN_topic, 
					apriltagList, 
					queue_size=10
					)
					
		# Publish to apriltagCenter msg
		self.apriltagCenterX_topic = "/isApriltag/Center/X"
		self.apriltagCenterX_pub = rospy.Publisher(
					self.apriltagCenterX_topic, 
					apriltagCenter, 
					queue_size=10
					)

		# Publish to apriltagCenter msg
		self.apriltagCenterY_topic = "/isApriltag/Center/Y"
		self.apriltagCenterY_pub = rospy.Publisher(
					self.apriltagCenterY_topic, 
					apriltagCenter, 
					queue_size=10
					)

		# Publish to apriltagHomography msg
		self.apriltagHomography_H00_topic = "/isApriltag/Homography/H00"
		self.apriltagHomography_H00_pub = rospy.Publisher(
					self.apriltagHomography_H00_topic, 
					apriltagHomography, 
					queue_size=10
					)

		# Publish to apriltagHomography msg
		self.apriltagHomography_H01_topic = "/isApriltag/Homography/H01"
		self.apriltagHomography_H01_pub = rospy.Publisher(
					self.apriltagHomography_H01_topic, 
					apriltagHomography, 
					queue_size=10
					)

		# Publish to apriltagHomography msg
		self.apriltagHomography_H02_topic = "/isApriltag/Homography/H02"
		self.apriltagHomography_H02_pub = rospy.Publisher(
					self.apriltagHomography_H02_topic, 
					apriltagHomography, 
					queue_size=10
					)

		# Publish to apriltagHomography msg
		self.apriltagHomography_H10_topic = "/isApriltag/Homography/H10"
		self.apriltagHomography_H10_pub = rospy.Publisher(
					self.apriltagHomography_H10_topic, 
					apriltagHomography, 
					queue_size=10
					)

		# Publish to apriltagHomography msg
		self.apriltagHomography_H11_topic = "/isApriltag/Homography/H11"
		self.apriltagHomography_H11_pub = rospy.Publisher(
					self.apriltagHomography_H11_topic, 
					apriltagHomography, 
					queue_size=10
					)

		# Publish to apriltagHomography msg
		self.apriltagHomography_H12_topic = "/isApriltag/Homography/H12"
		self.apriltagHomography_H12_pub = rospy.Publisher(
					self.apriltagHomography_H12_topic, 
					apriltagHomography, 
					queue_size=10
					)

		# Publish to apriltagHomography msg
		self.apriltagHomography_H20_topic = "/isApriltag/Homography/H20"
		self.apriltagHomography_H20_pub = rospy.Publisher(
					self.apriltagHomography_H20_topic, 
					apriltagHomography, 
					queue_size=10
					)

		# Publish to apriltagHomography msg
		self.apriltagHomography_H21_topic = "/isApriltag/Homography/H21"
		self.apriltagHomography_H21_pub = rospy.Publisher(
					self.apriltagHomography_H21_topic, 
					apriltagHomography, 
					queue_size=10
					)

		# Publish to apriltagHomography msg
		self.apriltagHomography_H22_topic = "/isApriltag/Homography/H22"
		self.apriltagHomography_H22_pub = rospy.Publisher(
					self.apriltagHomography_H22_topic, 
					apriltagHomography, 
					queue_size=10
					)

		# Publish to apriltagCorner msg
		self.apriltagCorner_X1_topic = "/isApriltag/Corner/X1"
		self.apriltagCorner_X1_pub = rospy.Publisher(
					self.apriltagCorner_X1_topic, 
					apriltagCorner, 
					queue_size=10
					)

		# Publish to apriltagCorner msg
		self.apriltagCorner_Y1_topic = "/isApriltag/Corner/Y1"
		self.apriltagCorner_Y1_pub = rospy.Publisher(
					self.apriltagCorner_Y1_topic, 
					apriltagCorner, 
					queue_size=10
					)

		# Publish to apriltagCorner msg
		self.apriltagCorner_X2_topic = "/isApriltag/Corner/X2"
		self.apriltagCorner_X2_pub = rospy.Publisher(
					self.apriltagCorner_X2_topic, 
					apriltagCorner, 
					queue_size=10
					)

		# Publish to apriltagCorner msg
		self.apriltagCorner_Y2_topic = "/isApriltag/Corner/Y2"
		self.apriltagCorner_Y2_pub = rospy.Publisher(
					self.apriltagCorner_Y2_topic, 
					apriltagCorner, 
					queue_size=10
					)

		# Publish to apriltagCorner msg
		self.apriltagCorner_X3_topic = "/isApriltag/Corner/X3"
		self.apriltagCorner_X3_pub = rospy.Publisher(
					self.apriltagCorner_X3_topic, 
					apriltagCorner, 
					queue_size=10
					)

		# Publish to apriltagCorner msg
		self.apriltagCorner_Y3_topic = "/isApriltag/Corner/Y3"
		self.apriltagCorner_Y3_pub = rospy.Publisher(
					self.apriltagCorner_Y3_topic, 
					apriltagCorner, 
					queue_size=10
					)

		# Publish to apriltagCorner msg
		self.apriltagCorner_X4_topic = "/isApriltag/Corner/X4"
		self.apriltagCorner_X4_pub = rospy.Publisher(
					self.apriltagCorner_X4_topic, 
					apriltagCorner, 
					queue_size=10
					)
					
		# Publish to apriltagCorner msg
		self.apriltagCorner_Y4_topic = "/isApriltag/Corner/Y4"
		self.apriltagCorner_Y4_pub = rospy.Publisher(
					self.apriltagCorner_Y4_topic, 
					apriltagCorner, 
					queue_size=10
					)
					
		# Publish to apriltagDistance msg
		self.apriltagDistance_topic = "/isApriltag/Distance"
		self.apriltagDistance_pub = rospy.Publisher(
					self.apriltagDistance_topic, 
					apriltagDistance, 
					queue_size=10
					)
					
		# Allow up to one second to connection
		rospy.sleep(1)

	# Convert image to OpenCV format
	def cbImage(self, msg):

		try:
			# direct conversion to cv2
			self.cv_image = self.bridge.compressed_imgmsg_to_cv2(
								msg, 
								"bgr8"
								)
		except CvBridgeError as e:
			print(e)

		if self.cv_image is not None:
			self.image_received = True
		else:
			self.image_received = False
			
	def cbAprilTag(self):
		# Info parameters configuration
		fontFace = cv2.FONT_HERSHEY_PLAIN
		fontScale = 0.7
		color = (255, 255, 255)
		colorPose = (0, 0, 255)
		colorIMU = (255, 0, 255)
		thickness = 1
		lineType = cv2.LINE_AA
		bottomLeftOrigin = False # if True (text upside down)

		# Converting to grayscale
		cv_image_gray = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)

		# AprilTag detection
		result = self.detector.detect(cv_image_gray)

		# Is any Apriltag detected?
		if len(result) != 0:
			self.isApriltag.data = True
			self.isApriltag_pub.publish(self.isApriltag)

			self.apriltagN_list = []

			self.apriltagCenterX_list = []
			self.apriltagCenterY_list = []

			self.apriltagHomography_H00_list = []
			self.apriltagHomography_H01_list = []
			self.apriltagHomography_H02_list = []
			self.apriltagHomography_H10_list = []
			self.apriltagHomography_H11_list = []
			self.apriltagHomography_H12_list = []
			self.apriltagHomography_H20_list = []
			self.apriltagHomography_H21_list = []
			self.apriltagHomography_H22_list = []

			self.apriltagCorner_X1_list = []
			self.apriltagCorner_Y1_list = []
			self.apriltagCorner_X2_list = []
			self.apriltagCorner_Y2_list = []
			self.apriltagCorner_X3_list = []
			self.apriltagCorner_Y3_list = []
			self.apriltagCorner_X4_list = []
			self.apriltagCorner_Y4_list = []

			self.apriltagDistance_list = []

			for i in range(len(result)):
				cv2.putText(
					self.cv_image, 
					"%d" % (result[i][1]), 
					(int(result[i][6][0]), int(result[i][6][1])), 
					fontFace, 
					fontScale * 5, 
					color, 
					thickness * 5, 
					lineType, 
					bottomLeftOrigin)

				cv2.line(
					self.cv_image, 
					(int(result[i][7][0][0]), int(result[i][7][0][1])), 
					(int(result[i][7][1][0]), int(result[i][7][1][1])), 
					(0, 0, 255), 
					10)

				cv2.line(
					self.cv_image, 
					(int(result[i][7][0][0]), int(result[i][7][0][1])), 
					(int(result[i][7][3][0]), int(result[i][7][3][1])), 
					(0, 255, 0), 
					10)

				cv2.line(
					self.cv_image, 
					(int(result[i][7][1][0]), int(result[i][7][1][1])), 
					(int(result[i][7][2][0]), int(result[i][7][2][1])), 
					(255, 0, 0), 
					10)

				cv2.line(
					self.cv_image, 
					(int(result[i][7][2][0]), int(result[i][7][2][1])), 
					(int(result[i][7][3][0]), int(result[i][7][3][1])), 
					(255, 0, 0), 
					10)

				cv2.circle(
					self.cv_image, 
					(int(result[i][6][0]), int(result[i][6][1])), 
					10, 
					(255, 0, 0), 
					-1)

				self.apriltagN_list.append(result[i][1])

				self.apriltagCenterX_list.append(result[i][6][0])
				self.apriltagCenterY_list.append(result[i][6][1])

				self.apriltagHomography_H00_list.append(result[i][5][0][0])
				self.apriltagHomography_H01_list.append(result[i][5][0][1])
				self.apriltagHomography_H02_list.append(result[i][5][0][2])
				self.apriltagHomography_H10_list.append(result[i][5][1][0])
				self.apriltagHomography_H11_list.append(result[i][5][1][1])
				self.apriltagHomography_H12_list.append(result[i][5][1][2])
				self.apriltagHomography_H20_list.append(result[i][5][2][0])
				self.apriltagHomography_H21_list.append(result[i][5][2][1])
				self.apriltagHomography_H22_list.append(result[i][5][2][2])

				self.apriltagCorner_X1_list.append(result[i][7][0][0])
				self.apriltagCorner_Y1_list.append(result[i][7][0][1])
				self.apriltagCorner_X2_list.append(result[i][7][1][0])
				self.apriltagCorner_Y2_list.append(result[i][7][1][1])
				self.apriltagCorner_X3_list.append(result[i][7][2][0])
				self.apriltagCorner_Y3_list.append(result[i][7][2][1])
				self.apriltagCorner_X4_list.append(result[i][7][3][0])
				self.apriltagCorner_Y4_list.append(result[i][7][3][1])

				# AprilTag3 Distance from Camera
				self.apriltagDistance_list.append(self.distance_to_camera(abs(result[i][7][1][0] - result[i][7][0][0])))

			self.isApriltagN.apriltagN = self.apriltagN_list
			self.isApriltagN_pub.publish(self.isApriltagN)

			self.apriltagCenterX.apriltagC = self.apriltagCenterX_list
			self.apriltagCenterY.apriltagC = self.apriltagCenterY_list
			self.apriltagCenterX_pub.publish(self.apriltagCenterX)
			self.apriltagCenterY_pub.publish(self.apriltagCenterY)

			self.apriltagH00.apriltagH = self.apriltagHomography_H00_list
			self.apriltagH01.apriltagH = self.apriltagHomography_H01_list
			self.apriltagH02.apriltagH = self.apriltagHomography_H02_list
			self.apriltagH10.apriltagH = self.apriltagHomography_H10_list
			self.apriltagH11.apriltagH = self.apriltagHomography_H11_list
			self.apriltagH12.apriltagH = self.apriltagHomography_H12_list
			self.apriltagH20.apriltagH = self.apriltagHomography_H20_list
			self.apriltagH21.apriltagH = self.apriltagHomography_H21_list
			self.apriltagH22.apriltagH = self.apriltagHomography_H22_list

			self.apriltagHomography_H00_pub.publish(self.apriltagH00)
			self.apriltagHomography_H01_pub.publish(self.apriltagH01)
			self.apriltagHomography_H02_pub.publish(self.apriltagH02)
			self.apriltagHomography_H10_pub.publish(self.apriltagH10)
			self.apriltagHomography_H11_pub.publish(self.apriltagH11)
			self.apriltagHomography_H12_pub.publish(self.apriltagH12)
			self.apriltagHomography_H20_pub.publish(self.apriltagH20)
			self.apriltagHomography_H21_pub.publish(self.apriltagH21)
			self.apriltagHomography_H22_pub.publish(self.apriltagH22)

			self.apriltagCornerX1.apriltagCorner = self.apriltagCorner_X1_list
			self.apriltagCornerY1.apriltagCorner = self.apriltagCorner_Y1_list
			self.apriltagCornerX2.apriltagCorner = self.apriltagCorner_X2_list
			self.apriltagCornerY2.apriltagCorner = self.apriltagCorner_Y2_list
			self.apriltagCornerX3.apriltagCorner = self.apriltagCorner_X3_list
			self.apriltagCornerY3.apriltagCorner = self.apriltagCorner_Y3_list
			self.apriltagCornerX4.apriltagCorner = self.apriltagCorner_X4_list
			self.apriltagCornerY4.apriltagCorner = self.apriltagCorner_Y4_list

			self.apriltagCorner_X1_pub.publish(self.apriltagCornerX1)
			self.apriltagCorner_Y1_pub.publish(self.apriltagCornerY1)
			self.apriltagCorner_X2_pub.publish(self.apriltagCornerX2)
			self.apriltagCorner_Y2_pub.publish(self.apriltagCornerY2)
			self.apriltagCorner_X3_pub.publish(self.apriltagCornerX3)
			self.apriltagCorner_Y3_pub.publish(self.apriltagCornerY3)
			self.apriltagCorner_X4_pub.publish(self.apriltagCornerX4)
			self.apriltagCorner_Y4_pub.publish(self.apriltagCornerY4)

			self.apriltagDistance.apriltagDistance = self.apriltagDistance_list
			self.apriltagDistance_pub.publish(self.apriltagDistance)

		else:
			# AprilTag Detected?
			self.isApriltag.data = False
			self.isApriltag_pub.publish(self.isApriltag)

			# What AprilTag Detected?
			self.apriltagN_list = []
			self.isApriltagN.apriltagN = self.apriltagN_list
			self.isApriltagN_pub.publish(self.isApriltagN)

			self.apriltagCenterX_list = []
			self.apriltagCenterY_list = []
			self.apriltagCenterX.apriltagC = self.apriltagCenterX_list
			self.apriltagCenterY.apriltagC = self.apriltagCenterY_list
			self.apriltagCenterX_pub.publish(self.apriltagCenterX)
			self.apriltagCenterX_pub.publish(self.apriltagCenterX)

			self.apriltagHomography_H00_list = []
			self.apriltagHomography_H01_list = []
			self.apriltagHomography_H02_list = []
			self.apriltagHomography_H10_list = []
			self.apriltagHomography_H11_list = []
			self.apriltagHomography_H12_list = []
			self.apriltagHomography_H20_list = []
			self.apriltagHomography_H21_list = []
			self.apriltagHomography_H22_list = []

			self.apriltagH00.apriltagH = self.apriltagHomography_H00_list
			self.apriltagH01.apriltagH = self.apriltagHomography_H01_list
			self.apriltagH02.apriltagH = self.apriltagHomography_H02_list
			self.apriltagH10.apriltagH = self.apriltagHomography_H10_list
			self.apriltagH11.apriltagH = self.apriltagHomography_H11_list
			self.apriltagH12.apriltagH = self.apriltagHomography_H12_list
			self.apriltagH20.apriltagH = self.apriltagHomography_H20_list
			self.apriltagH21.apriltagH = self.apriltagHomography_H21_list
			self.apriltagH22.apriltagH = self.apriltagHomography_H22_list

			self.apriltagHomography_H00_pub.publish(self.apriltagH00)
			self.apriltagHomography_H01_pub.publish(self.apriltagH01)
			self.apriltagHomography_H02_pub.publish(self.apriltagH02)
			self.apriltagHomography_H10_pub.publish(self.apriltagH10)
			self.apriltagHomography_H11_pub.publish(self.apriltagH11)
			self.apriltagHomography_H12_pub.publish(self.apriltagH12)
			self.apriltagHomography_H20_pub.publish(self.apriltagH20)
			self.apriltagHomography_H21_pub.publish(self.apriltagH21)
			self.apriltagHomography_H22_pub.publish(self.apriltagH22)

			self.apriltagCorner_X1_list = []
			self.apriltagCorner_Y1_list = []
			self.apriltagCorner_X2_list = []
			self.apriltagCorner_Y2_list = []
			self.apriltagCorner_X3_list = []
			self.apriltagCorner_Y3_list = []
			self.apriltagCorner_X4_list = []
			self.apriltagCorner_Y4_list = []

			self.apriltagCornerX1.apriltagCorner = self.apriltagCorner_X1_list
			self.apriltagCornerY1.apriltagCorner = self.apriltagCorner_Y1_list
			self.apriltagCornerX2.apriltagCorner = self.apriltagCorner_X2_list
			self.apriltagCornerY2.apriltagCorner = self.apriltagCorner_Y2_list
			self.apriltagCornerX3.apriltagCorner = self.apriltagCorner_X3_list
			self.apriltagCornerY3.apriltagCorner = self.apriltagCorner_Y3_list
			self.apriltagCornerX4.apriltagCorner = self.apriltagCorner_X4_list
			self.apriltagCornerY4.apriltagCorner = self.apriltagCorner_Y4_list

			self.apriltagCorner_X1_pub.publish(self.apriltagCornerX1)
			self.apriltagCorner_Y1_pub.publish(self.apriltagCornerY1)
			self.apriltagCorner_X2_pub.publish(self.apriltagCornerX2)
			self.apriltagCorner_Y2_pub.publish(self.apriltagCornerY2)
			self.apriltagCorner_X3_pub.publish(self.apriltagCornerX3)
			self.apriltagCorner_Y3_pub.publish(self.apriltagCornerY3)
			self.apriltagCorner_X4_pub.publish(self.apriltagCornerX4)
			self.apriltagCorner_Y4_pub.publish(self.apriltagCornerY4)

			self.apriltagDistance_list = []
			self.apriltagDistance.apriltagDistance = self.apriltagDistance_list
			self.apriltagDistance_pub.publish(self.apriltagDistance)

		cv2.putText(
			self.cv_image, 
			"N AprilTag3: %d" % (len(self.apriltagN_list)), 
			(20, 40), 
			fontFace, 
			fontScale * 4, 
			(0, 0, 255), 
			thickness * 2, 
			lineType, 
			bottomLeftOrigin)
			
		cv2.putText(
			self.cv_image, 
			"AprilTag3 List: %s" % (self.apriltagN_list), 
			(20, 80), 
			fontFace, 
			fontScale * 4, 
			(0, 0, 255), 
			thickness * 2, 
			lineType, 
			bottomLeftOrigin)
			
	def distance_to_camera(self, perWidth):
		# compute and return the distance from the maker to the camera
		return (self.knownWidth * self.focalLength) / perWidth

	# Show the output frame
	def cbShowImage(self):
		self.cv_image_clone = imutils.resize(
					self.cv_image.copy(), 
					width=320
					)
					
		cv2.imshow("AprilTag Detection", self.cv_image_clone)
		cv2.waitKey(1)
		
	# Preview image + info
	def cbPreview(self):
		if self.image_received:
			self.cbAprilTag()
			self.cbShowImage()
		else:
			rospy.logerr("No images recieved")
			
	# rospy shutdown callback
	def cbShutdown(self):
		rospy.logerr("AprilTag Detection Node [OFFLINE]...")
		cv2.destroyAllWindows()

if __name__ == '__main__':
	# Initialize
	rospy.init_node('camera_apriltag_detection', anonymous=False)
	camera = CameraAprilTag()
	
	r = rospy.Rate(10)
	
	# Camera preview
	while not rospy.is_shutdown():
		camera.cbPreview()
		r.sleep()

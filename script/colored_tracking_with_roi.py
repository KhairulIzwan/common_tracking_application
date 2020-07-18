#!/usr/bin/env python

#Title: Python Subscriber for Tank Navigation
#Author: Khairul Izwan Bin Kamsani - [23-01-2020]
#Description: Tank Navigation Subcriber Nodes (Python)

from __future__ import print_function
from __future__ import division

# import the necessary packages
from imutils import face_utils
from collections import deque
import imutils
import time
import cv2
import os
import rospkg
import sys
import rospy
import numpy as np

# import the necessary ROS messages
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

from cv_bridge import CvBridge
from cv_bridge import CvBridgeError

#from sensor_msgs.msg import RegionOfInterest

from common_tracking_application.objcenter import objCenter
from common_face_application.msg import objCenter as objCoord

class ColoredTracking:

	def __init__(self, buffer=16):

		rospy.logwarn("ColoredTracking (ROI) node [ONLINE]")

		self.bridge = CvBridge()
#		self.roi = RegionOfInterest()
		self.objectCoord = objCoord()

		# define the lower and upper boundaries of the "green"
		# ball in the HSV color space, then initialize the
		# list of tracked points
		self.greenLower = (29, 86, 6)
		self.greenUpper = (64, 255, 255)
		self.pts = deque(maxlen=buffer)
		self.buffer = buffer

		self.image_recieved = False

		# rospy shutdown
		rospy.on_shutdown(self.cbShutdown)

		# Subscribe to Image msg
		image_topic = "/cv_camera/image_raw"
		self.image_sub = rospy.Subscriber(image_topic, Image, self.cbImage)

		# Subscribe to CameraInfo msg
		cameraInfo_topic = "/cv_camera/camera_info"
		self.cameraInfo_sub = rospy.Subscriber(cameraInfo_topic, CameraInfo, 
			self.cbCameraInfo)

#		# Publish to RegionOfInterest msg
#		roi_topic = "/faceROI"
#		self.roi_pub = rospy.Publisher(roi_topic, RegionOfInterest, queue_size=10)

		# Publish to objCenter msg
		objCoord_topic = "/objCoord"
		self.objCoord_pub = rospy.Publisher(objCoord_topic, objCoord, queue_size=10)

		# Allow up to one second to connection
		rospy.sleep(1)

	# Convert image to OpenCV format
	def cbImage(self, msg):

		try:
			self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
		except CvBridgeError as e:
			print(e)

		if self.cv_image is not None:
			self.image_received = True
		else:
			self.image_received = False

	# Get CameraInfo
	def cbCameraInfo(self, msg):

		self.imgWidth = msg.width
		self.imgHeight = msg.height

		# calculate the center of the frame as this is where we will
		# try to keep the object
		self.centerX = self.imgWidth // 2
		self.centerY = self.imgHeight // 2
		self.centerY = np.int0(0.9 * self.imgHeight)

	# Show the output frame
	def cbShowImage(self):

		cv2.imshow("Haar Face Detector (ROI)", self.cv_image)
		cv2.waitKey(1)

	# Image information callback
	def cbInfo(self):

		fontFace = cv2.FONT_HERSHEY_DUPLEX
		fontScale = 0.5
		color = (255, 255, 255)
		thickness = 1
		lineType = cv2.LINE_AA
		bottomLeftOrigin = False # if True (text upside down)

		self.timestr = time.strftime("%Y%m%d-%H:%M:%S")

		cv2.putText(self.cv_image, "{}".format(self.timestr), (10, 20), 
			fontFace, fontScale, color, thickness, lineType, 
			bottomLeftOrigin)
		cv2.putText(self.cv_image, "(%d, %d)" % (self.objX, self.objY), 
			(self.imgWidth-100, 20), fontFace, fontScale, 
			color, thickness, lineType, bottomLeftOrigin)
		cv2.putText(self.cv_image, "Sample", (10, self.imgHeight-10), 
			fontFace, fontScale, color, thickness, lineType, 
			bottomLeftOrigin)
		cv2.putText(self.cv_image, "(%d, %d)" % (self.imgWidth, self.imgHeight), 
			(self.imgWidth-100, self.imgHeight-10), fontFace, fontScale, 
			color, thickness, lineType, bottomLeftOrigin)

	# Detect the face(s)
	def cbFace(self):
		if self.image_received:
			# resize the frame, blur it, and convert it to the HSV
			# color space
			frame = imutils.resize(self.cv_image, width=self.imgWidth)
			blurred = cv2.GaussianBlur(self.cv_image, (11, 11), 0)
			hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
			
			# construct a mask for the color "green", then perform
			# a series of dilations and erosions to remove any small
			# blobs left in the mask
			mask = cv2.inRange(hsv, self.greenLower, self.greenUpper)
			mask = cv2.erode(mask, None, iterations=2)
			mask = cv2.dilate(mask, None, iterations=2)
	
			# find contours in the mask and initialize the current
			# (x, y) center of the ball
			cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
				cv2.CHAIN_APPROX_SIMPLE)
			cnts = imutils.grab_contours(cnts)
			
			# only proceed if at least one contour was found
			objectLoc = objCenter(cnts, (self.centerX, self.centerY))
			((self.objX, self.objY), rect) = objectLoc
			
			self.pubObjCoord()
			
			# extract the bounding box and draw it
			if rect is not None:
				cv2.circle(self.cv_image, (int(self.objX), int(self.objY)), 
					int(rect), (0, 255, 255), 2)

			self.cbInfo()
			self.cbShowImage()
			
			# Allow up to one second to connection
			rospy.sleep(0.1)
		else:
			rospy.logerr("No images recieved")

#	# Publish to RegionOfInterest msg
#	def pubRegionofInterest(self):

#		self.roi.x_offset = self.x
#		self.roi.y_offset = self.y
#		self.roi.width = self.x + self.w
#		self.roi.height = self.y + self.h

#		self.roi_pub.publish(self.roi)

	# Publish to objCenter msg
	def pubObjCoord(self):

		self.objectCoord.centerX = self.objX
		self.objectCoord.centerY = self.objY

		self.objCoord_pub.publish(self.objectCoord)

	# rospy shutdown callback
	def cbShutdown(self):
		try:
			rospy.logwarn("ColoredTracking (ROI) node [OFFLINE]")
		finally:
			cv2.destroyAllWindows()

if __name__ == '__main__':

	# Initializing your ROS Node
	rospy.init_node('colored_tracking', anonymous=False)
	color = ColoredTracking()

	# Camera preview
	while not rospy.is_shutdown():
		color.cbFace()

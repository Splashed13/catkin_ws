#!/usr/bin/env python

import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError

class ImageProcessor():
	def __init__(self):
		# Get the path to the cascade XML training file using ROS parameters
		# Then load in our cascade classifier
		sign_cascade_file = str(rospy.get_param("~cascade_file"))
		self.sign_cascade = cv2.CascadeClassifier(sign_cascade_file)

		# Set up the CV Bridge
		self.bridge = CvBridge()

		# Set up the subscriber
		self.sub_img = rospy.Subscriber('~input/image_raw/compressed', CompressedImage, self.callback_img)
		self.pub_img = rospy.Publisher('~output/image_raw/compressed', CompressedImage, queue_size=1)

	def shutdown(self):
		# Unregister anything that needs it here
		self.sub_img.unregister()

	def callback_img(self, msg_in):
		# Convert ROS image to CV image
		try:
			cv_image = self.bridge.compressed_imgmsg_to_cv2( msg_in, "bgr8" )
		except CvBridgeError as e:
			print(e)

		# ===================
		# Do processing here!
		# ===================
		gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

		sign = self.sign_cascade.detectMultiScale(gray, 1.01, 1, minSize=(100,100))

		for (x,y,w,h) in sign:
			cv2.rectangle(cv_image,(x,y),(x+w,y+h),(255,0,0),2)
		# ===================

		# Convert CV image to ROS image and publish
		try:
			self.pub_img.publish( self.bridge.cv2_to_compressed_imgmsg( cv_image ) )
		except CvBridgeError as e:
			print(e)

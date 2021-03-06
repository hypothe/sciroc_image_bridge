#!/usr/bin/env python
from __future__ import print_function

import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import os

class image_converter:

	def __init__(self):
		self.image_pub = rospy.Publisher("/synth_camera/image_raw",Image, queue_size=10)

		self.bridge = CvBridge()
		self.imgmsg = Image()

	def start_timer(self):
		self.timer = rospy.Timer(rospy.Duration(0, 1000000000/30), self.periodic_pub)
		
	def get_image_from_file(self, path_to_img):
		assert os.path.isfile(path_to_img), "ERROR: file %s not found" % path_to_img

		img = cv2.imread(path_to_img)
		if img is None:
			sys.exit("ERROR: could not read image %s" % path_to_img)

		(rows,cols,channels) = img.shape
		# cv2.imshow("Input image %sx%sx%s" % (rows,cols,channels), img)
		# cv2.waitKey(3000)

		try:
			self.imgmsg = self.bridge.cv2_to_imgmsg(img, "bgr8")
		except CvBridgeError as e:
			print(e)

	def periodic_pub(self, event):	
		self.image_pub.publish(self.imgmsg)
#
#
################################ Class : end

def main(args):
	ic = image_converter()
	rospy.init_node('image_repub', anonymous=True)
	rate = rospy.Rate(1)

	print("Insert image path/name:\t")
	path_to_img = input()
	ic.get_image_from_file(path_to_img)
	ic.start_timer()

	while not rospy.is_shutdown():
		print("Insert new image path/name:\t")
		path_to_img = input()
		ic.get_image_from_file(path_to_img)

		rate.sleep()

	cv2.destroyAllWindows()

if __name__ == '__main__':
		main(sys.argv)
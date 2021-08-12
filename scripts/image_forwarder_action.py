#!/usr/bin/env python
from __future__ import print_function

import sys
import rospy
import actionlib
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import os
import darknet_ros_msgs.msg

class image_converter:

	def __init__(self):
		self.image_pub = rospy.Publisher("/camera/rgb/image_raw",Image, queue_size=10)

		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/darknet_ros/detection_image",Image, self.callback)
		self.onehot = False
		self.count = 0

		self.ac_ = actionlib.SimpleActionClient('image_bridge', darknet_ros_msgs.msg.CheckForObjectsAction)
		self.ac_.wait_for_server()

	#def callback(self,data):
	#	if not self.onehot:
	#		return
#
	#	try:
	#		cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
	#	except CvBridgeError as e:
	#		print(e)
#
	#	cv2.imshow("Image after YOLO", cv_image)
	#	cv2.waitKey(6000)
	#	self.onehot = False
		
	def image_forward(self, path_to_img):
		assert os.path.isfile(path_to_img), "ERROR: file %s not found" % path_to_img

		img = cv2.imread(path_to_img)
		if img is None:
			sys.exit("ERROR: could not read image %s" % path_to_img)

		(rows,cols,channels) = img.shape
		#cv2.imshow("Input image %sx%sx%s" % (rows,cols,channels), img)
		#cv2.waitKey(3000)

		try:
			goal = darknet_ros_msgs.msg.CheckForObjectsActionGoal(id = self.count, \
																														image = self.bridge.cv2_to_imgmsg(img, "rgb8"))
			self.ac_.send_goal(goal)
			
			self.ac_.wait_for_result(timeout=rospy.Duration(secs=5))

			for box in self.ac_.get_result().bounding_boxes:
				print("Class: %s\n\tprob: %ld\n\tmin: [%ld, %ld]\n\tmax: [%ld, %ld]" \
					% (box.Class, box.probability, box.xmin, box.ymin, box.xmax, box.ymax))

		except CvBridgeError as e:
			print(e)
#
#
################################ Class : end

def main(args):
	ic = image_converter()
	rospy.init_node('image_converter', anonymous=True)
	rate = rospy.Rate(1)

	while not rospy.is_shutdown():
		print("Insert image path/name:\t")
		path_to_img = input()
		ic.image_forward(path_to_img)

		rate.sleep()

	cv2.destroyAllWindows()

if __name__ == '__main__':
		main(sys.argv)
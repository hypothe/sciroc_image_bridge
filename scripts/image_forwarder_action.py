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

from timeit import default_timer as timer

class image_converter:

	def __init__(self):

		self.bridge = CvBridge()
		self.count = 0

		self.ac_ = actionlib.SimpleActionClient('darknet_ros_as/check_for_objects', darknet_ros_msgs.msg.CheckForObjectsAction)
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
		if not os.path.isfile(path_to_img):
			print("ERROR: file %s not found" % path_to_img)
			return

		img = cv2.imread(path_to_img)
		if img is None:
			sys.exit("ERROR: could not read image %s" % path_to_img)

		(rows,cols,channels) = img.shape
		#cv2.imshow("Input image %sx%sx%s" % (rows,cols,channels), img)
		#cv2.waitKey(3000)

		try:
			goal = darknet_ros_msgs.msg.CheckForObjectsActionGoal().goal
			goal.id = self.count
			goal.image = self.bridge.cv2_to_imgmsg(img, "bgr8")
			
			start = timer()
			self.ac_.send_goal(goal)
			self.count += 1
			ret = self.ac_.wait_for_result(timeout=rospy.Duration(secs=20))
			end = timer()

			elapsed = end - start
			if not ret:
				print("NO RESPONSE")
				return

			result = self.ac_.get_result()
			print("Elapsed time %f" % elapsed)
			for box in result.bounding_boxes.bounding_boxes:
				print("Class: %s\n\tprob: %lf\n\tmin: [%ld, %ld]\n\tmax: [%ld, %ld]" \
					% (box.Class, box.probability, box.xmin, box.ymin, box.xmax, box.ymax))

		except CvBridgeError as e:
			print(e)
#
#
################################ Class : end

def main(args):
	rospy.init_node('image_converter', anonymous=True)
	ic = image_converter()
	rate = rospy.Rate(1)

	while not rospy.is_shutdown():
		print("Insert image path/name:\t")
		path_to_img = input()
		ic.image_forward(path_to_img)

		rate.sleep()

	cv2.destroyAllWindows()

if __name__ == '__main__':
		main(sys.argv)
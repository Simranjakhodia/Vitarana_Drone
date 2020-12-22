#!/usr/bin/env python


'''
This is a boiler plate script that contains an example on how to subscribe a rostopic containing camera frames 
and store it into an OpenCV image to use it further for image processing tasks.
Use this code snippet in your code or you can also continue adding your code in the same file
'''

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import rospy
from pyzbar.pyzbar import decode
from sensor_msgs.msg import Imu,NavSatFix
from std_msgs.msg import String
# import rosservice
from vitarana_drone.srv import Gripper
import ast
class image_proc():

	# Initialise everything
	def __init__(self):
		rospy.init_node('barcode_test') #Initialise rosnode 
		self.image_sub = rospy.Subscriber("/edrone/camera/image_raw", Image, self.image_callback) #Subscribing to the camera topic
		self.img = np.empty([]) # This will contain your image frame from camera
		self.bridge = CvBridge()
		self.latitude=0.0
		self.longitude=0.0
		self.altitude=0.0
		self.alt = 0.0
		self.val = False
		# rospy.Subscriber('/edrone/gps', NavSatFix , self.gps_set_pid)
		rospy.Subscriber('/edrone/gripper_check',String,self.check)
	def check(self,data):
		print(data.data,self.alt)
		self.val = ast.literal_eval(data.data)
		value = self.val
		# rospy.wait_for_service("edrone/activate_gripper")
		if value == True:
			try:
			    grip = rospy.ServiceProxy("edrone/activate_gripper",Gripper)
			    x = grip.call(True)
			    print(x)
			except rospy.ServiceException as e:
			    print("Service call failed: %s"%e)

	def gps_set_pid(self,msg):
		self.alt = msg.altitude
	# Callback function of amera topic
	def image_callback(self, data):
		try:
			self.img = self.bridge.imgmsg_to_cv2(data, "bgr8") # Converting the image to OpenCV standard image
			for barcode in decode(self.img):
				mycode = barcode.data.decode('UTF-8')
				encoded_data=mycode.encode('utf-8')
				coord=list(encoded_data.split(','))
				self.latitude=coord[0]
				self.longitude=coord[1]
				self.altitude=coord[2]
				print(self.alt)
			
		except CvBridgeError as e:
			print(e)
			print("hello")
			return

if __name__ == '__main__':
    image_proc_obj = image_proc()
    rospy.spin()


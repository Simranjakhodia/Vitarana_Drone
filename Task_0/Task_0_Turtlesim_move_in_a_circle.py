#!/usr/bin/env python
## VD_2241 
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import time
import sys
robot_x = 0
robot_y = 0
PI = 3.1415926535897
choice =1
def node_turtle_revolve(lin_vel, ang_vel):
    global robot_x, robot_y, present_time,choice

    rospy.init_node('node_turtle_revolve', anonymous=False)
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

    rospy.Subscriber('/turtle1/pose', Pose)

    rate = rospy.Rate(100)  # 100hz
    rad = float(lin_vel/ang_vel)
    dist = 2 * 3.14 * rad
    vel = Twist()
    zero_vel = Twist()
    vel.linear.x = lin_vel
    vel.angular.z = ang_vel
    start_time = rospy.Time.now()

    #
    while(choice!=0):#not rospy.is_shutdown(): 
        #calculating the angular distance covered by the robot
        #theta = w*t   # theta = W(omega) * t (time)
	#####
	present_time = rospy.Time.now()
	theta = vel.angular.z * (present_time - start_time)
	distance = (theta*rad)
	#####
        if rospy.Time.now() - start_time < rospy.Duration((2*math.pi/ang_vel)+0.2):
            	rospy.loginfo("Moving in a circle")
		##print(distance)
		print((rospy.Time.now()-start_time).to_sec())
		pub.publish(vel)
        else:
		choice = choice-1		
		rospy.loginfo("goal reached")            	
		#rotate()   #############
		# Receiveing the user's input
       		speed = 30
       		angle = 60
       		clockwise = False
 
       		#Converting from angles to radians
      		angular_speed = speed*2*PI/360
       		relative_angle = angle*2*PI/360
   
      		#We wont use linear components
       		vel.linear.x=0
       		vel.linear.y=0
       		vel.linear.z=0
       		vel.angular.x = 0
      		vel.angular.y = 0
   
       		# Checking if our movement is CW or CCW
       		if clockwise:
           		vel.angular.z = -abs(angular_speed)
       		else:
           		vel.angular.z = abs(angular_speed)
       		# Setting the current time for distance calculus
       		t0 = rospy.Time.now().to_sec()
       		current_angle = 0
   
       		while(current_angle < relative_angle):
           		pub.publish(vel)
           		t1 = rospy.Time.now().to_sec()
          		current_angle = angular_speed*(t1-t0)
   
   
       		#Forcing our robot to stop
       		vel.angular.z = 0
       		pub.publish(vel)
       		rospy.spin()
		#######################
		pub.publish(zero_vel)
		
        rate.sleep()
 

if __name__ == "__main__":
    try:
        node_turtle_revolve(3.0, 1.5)
    except rospy.ROSInterruptException:
        pass

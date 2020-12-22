#!/usr/bin/env python

# Team Leader- Shyam Marjit
# Team_Id -2241
# IIITG-8927860176

'''
This python file runs a ROS-node of name position_control which gives us the required output of the eDrone_task-1.
This node publishes and subsribes the following topics:
        PUBLICATIONS                SUBSCRIPTIONS
	/edrone/drone_command           /edrone/gps
Rather than using different variables, use list. eg : self.setpoint = [1,2,3], where index corresponds to x,y,z ...rather than defining self.x_setpoint = 1, self.y_setpoint = 2
CODE MODULARITY AND TECHNIQUES MENTIONED LIKE THIS WILL HELP YOU GAINING MORE MARKS WHILE CODE EVALUATION.
'''

# Importing the required libraries

from vitarana_drone.msg import *
from pid_tune.msg import PidTune
from sensor_msgs.msg import Imu,NavSatFix
from std_msgs.msg import Float32
import rospy
import time
import tf
import math


class Edrone():
    """docstring for Edrone"""
    def __init__(self):

        rospy.init_node('position_controller')  # Initializing ros node with name drone_control

	# Initial setting of Kp, Kd, Ki for [ pitch, roll, throttle ] after tuning
        self.Kp = [2860*0.06, 1467*0.06, 1.12 ]		# kp corresponds in order of pitch, roll, throttle
        self.Ki = [0.0 , 0.0, 0.000075 ]		# ki corresponds in order of pitch, roll, throttle
        self.Kd = [5000*100 , 5000*100, 1351.5 ]	# kd corresponds in order of pitch, roll, throttle

	# Variables for error computing 
        self.error = [0.0,0.0,0.0]  		# Errors in each axis
        self.prev_error = [0.0, 0.0, 0.0] 	# Previous errors in each axis
	self.errSum = [0.0, 0.0, 0.0]		# Integral error in each axis
        self.dErr = [0.0, 0.0, 0.0]		# Derivative error in each axis

	# Initinalizing latitude, longitude and altitude
        self.lat = 0.0
        self.lon = 0.0
        self.alt = 0.0

        self.fix_lat = 19.0
        self.fix_lon = 72.0
        self.fix_alt=  3.0

	self.prev_lat = 0.0
        self.prev_lon = 0.0
        self.prev_alt = 0.0
	
	# For storing Pid output for each axis
        self.out_roll = 0.0
        self.out_pitch = 0.0
        self.out_throttle = 0.0

	# Looping-variables
        self.loop_for_top = 1
        self.loop_for_right = 1

        # This is the sample time in which you need to run pid. Choose any time which you seem fit. Remember the stimulation step time is 30 ms
        self.sample_time = 30  # in mseconds
	self.drone_cmd = edrone_cmd()

        # Publishing /edrone/pwm, /roll_error, /pitch_error
        self.drone_pub = rospy.Publisher('/drone_command', edrone_cmd, queue_size=1)
        self.roll_pub = rospy.Publisher('/roll_error', Float32, queue_size=1)
        self.pitch_pub = rospy.Publisher('/pitch_error', Float32, queue_size=1)

        # Subscribing to /pid_tuning_roll, /pid_tuning_pitch, /pid_tuning_yaw, /pid_tuning_altitude, /edrone/gps
        rospy.Subscriber('/pid_tuning_roll', PidTune, self.roll_set_pid)
       	rospy.Subscriber('/pid_tuning_pitch', PidTune, self.pitch_set_pid)
        rospy.Subscriber('/edrone/gps', NavSatFix , self.gps_set_pid)

    #---------------------------------------------------------------------------------------------------------------------------------------------------------

    def gps_set_pid(self,msg):	# Function which recive gps values
        self.alt = msg.altitude
        self.lat = msg.latitude
        self.lon = msg.longitude
 

    # Callback function for /pid_tuning_roll
    # This function gets executed each time when /tune_pid publishes /pid_tuning_roll

    def roll_set_pid(self, roll):
        self.Kp[1] = roll.Kp * 0.06  	# This is just for an example. You can change the ratio/fraction value accordingly
        self.Ki[1] = roll.Ki * 0.0008
        self.Kd[1] = roll.Kd * 100
    
    def pitch_set_pid(self, pitch):
        self.Kp[0] = pitch.Kp * 0.06  	# This is just for an example. You can change the ratio/fraction value accordingly
        self.Ki[0] = pitch.Ki * 0.0008
        self.Kd[0] = pitch.Kd * 100

    #----------------------------------------------------------------------------------------------------------------------------------------------------------
    

    def pid(self):
	
	#---------------------------------------------Computing error in each axis--------------------------------------------------------------------------

        self.error[0] = (self.fix_lat-self.lat)*100000	# for roll axis
        self.error[1] = (self.fix_lon-self.lon)*100000	# for pitch axis
        self.error[2] = self.fix_alt - self.alt		# for throttle axis

	#----------------------------------------------loop-control for different set-points --------------------------------------------------------------------

        if self.loop_for_top == 0:
            if self.loop_for_right:
                if abs(self.error[0]/100000) < 0.000002500:
                    self.fix_alt = 0.3
                    self.loop_for_right = 0
        
        if self.loop_for_top:
            if abs(self.error[2] < 0.2):
                self.fix_lat = 19.0000451704
                self.loop_for_top = 0
 
        #---------------------------------------------Computing derivative error and Integral error in each axis----------------------------------------------
	
	# Computing derivative error for each axis
	self.dErr[0] = (self.error[0] - self.prev_error[0]) / self.sample_time	# for roll axis
	self.dErr[1] = (self.error[1] - self.prev_error[1]) / self.sample_time	# for pitch axis
	self.dErr[2] = (self.error[2] - self.prev_error[2]) / self.sample_time	# for throttle 

	# Computing integral error for each axis
        self.errSum[0] = self.errSum[0] + (self.error[0] * self.sample_time)	# for roll axis
        self.errSum[1] = self.errSum[1] + (self.error[1] * self.sample_time)	# for pitch axis
        self.errSum[2] = self.errSum[2] + (self.error[2] * self.sample_time)	# for throttle
        
	#---------------------------------------------Computing PID output for each axis-----------------------------------------------------------------------

        self.out_pitch = 1500 + (self.Kp[0] * self.error[1] + self.Ki[0] * self.errSum[1] + self.Kd[0] * self.dErr[1])      # for pitch  
        self.out_roll =  1500 +  (self.Kp[1] * self.error[0] + self.Ki[1] * self.errSum[0] + self.Kd[1] * self.dErr[0])     # for roll
        self.out_throttle = self.Kp[2] * self.error[2] + self.Ki[2] * self.errSum[2] + self.Kd[2] * self.dErr[2]	    # for throttle

	#-----------------------------------------------Storing previous values for next execution-----------------------------------------------------------

        self.prev_error[0] = self.error[0]	# for roll axis
        self.prev_error[1] = self.error[1]	# for pitch axis
	self.prev_error[2] = self.error[2]	# for throttle

	#-----------------------------------------------Limiting the pid output values for each axis------------------------------------------------------------------------------- 
	
	# For Roll axis       
        if self.out_roll > 2000:
            self.out_roll= 2000
	if self.out_roll < 1000:
            self.out_roll= 1000
	
	# For Pitch axis
        if self.out_pitch > 2000:
            self.out_pitch = 2000
        if self.out_pitch < 1000:
            self.out_pitch = 1000
	
	# For Throttle axis
        if self.out_throttle > 2000:
            self.out_throttle = 2000
        

	#-------------------------------------------------Giving the values to drone_cmd i.e rcRoll, rcPitch, rcThrottle and rcYaw-----------------------------------

        self.drone_cmd.rcYaw = 1500.0
        self.drone_cmd.rcRoll = self.out_roll
        self.drone_cmd.rcPitch = self.out_pitch
        self.drone_cmd.rcThrottle = self.out_throttle

	#------------------------------------------------------------------------------------------------------------------------------------------------------------
        self.drone_pub.publish(self.drone_cmd)
        


if __name__ == '__main__':

    e_drone = Edrone()
    r = rospy.Rate(e_drone.sample_time)  # specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
    while not rospy.is_shutdown():
        e_drone.pid()
        r.sleep()

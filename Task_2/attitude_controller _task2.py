#!/usr/bin/env python

'''
This python file runs a ROS-node of name attitude_control which controls the roll pitch and yaw angles of the eDrone.
This node publishes and subsribes the following topics:
        PUBLICATIONS            SUBSCRIPTIONS
        /roll_error             /pid_tuning_altitude
        /pitch_error            /pid_tuning_pitch
        /yaw_error              /pid_tuning_roll
        /edrone/pwm             /edrone/imu/data
                                /edrone/drone_command
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
        rospy.init_node('attitude_controller')  # initializing ros node with name drone_control

        # This corresponds to your current orientation of eDrone in quaternion format. This value must be updated each time in your imu callback
        # [x,y,z,w]
        self.drone_orientation_quaternion = [0.0, 0.0, 0.0, 0.0]

        # This corresponds to your current orientation of eDrone converted in euler angles form.
        # [r,p,y]
        self.drone_orientation_euler = [0.0, 0.0, 0.0]

        # This is the setpoint that will be received from the drone_command in the range from 1000 to 2000
        # [r_setpoint, p_setpoint, y_setpoint]
        self.setpoint_cmd = [1500.0, 1500.0, 1500.0]

        # The setpoint of orientation in euler angles at which you want to stabilize the drone
        # [r_setpoint, p_psetpoint, y_setpoint]
        self.setpoint_euler = [0.0, 0.0, 0.0]
        self.setpoint_throttle = 0
        self.pwm_cmd = prop_speed()
        self.pwm_cmd.prop1 = 0.0
        self.pwm_cmd.prop2 = 0.0
        self.pwm_cmd.prop3 = 0.0
        self.pwm_cmd.prop4 = 0.0

        self.out_base = 0

        # initial setting of Kp, Kd and ki for [roll, pitch, yaw]. eg: self.Kp[2] corresponds to Kp value in yaw axis
        # after tuning and computing corresponding PID parameters, change the parameters
        self.Kp = [5.22, 4, 200.0, 1.12]
        self.Ki = [0.0, 0.0, 0.0, 0.000075]
        self.Kd = [1454 , 1382.0, 3000.0, 1351.5]
        self.error = [0.0, 0.0, 0.0, 0.0]  #errors in each axis
        self.prev_error = [0.0, 0.0, 0.0, 0.0] #previous errors in each axis
      #  self.max_values = [256, 256, 256, 256]  #max values
      #  self.min_values = [0, 0, 0, 0]              #min values
        self.out_roll = 0.0
        self.out_pitch = 0.0
        self.out_yaw = 0.0
        self.out_throttle = 0.0
        self.errSum = [0.0, 0.0, 0.0, 0.0]
        self.dErr = [0.0, 0.0, 0.0, 0.0]
        # This is the sample time in which you need to run pid. Choose any time which you seem fit. Remember the stimulation step time is 50 ms
        self.sample_time = 30  # in mseconds
        self.fix_alt = 3.0
        self.alt = 0
        # Publishing /edrone/pwm, /roll_error, /pitch_error, /yaw_error
        self.pwm_pub = rospy.Publisher('/edrone/pwm', prop_speed, queue_size=1)
      #  self.roll_pub = rospy.Publisher('/roll_error', Float32, queue_size=1)
       # self.pitch_pub = rospy.Publisher('/pitch_error', Float32, queue_size=1)
       # self.yaw_pub = rospy.Publisher('/yaw_error', Float32, queue_size=1)
        self.alt_pub = rospy.Publisher('/z_error', Float32, queue_size=1)

        # Subscribing to /drone_command, imu/data, /pid_tuning_roll, /pid_tuning_pitch, /pid_tuning_yaw
        rospy.Subscriber('/drone_command', edrone_cmd, self.drone_command_callback)
        rospy.Subscriber('/edrone/imu/data', Imu, self.imu_callback)
        # rospy.Subscriber('/pid_tuning_roll', PidTune, self.roll_set_pid)
        #rospy.Subscriber('/edrone/gps', NavSatFix , self.gps_set_pid)
        # rospy.Subscriber('/pid_tuning_altitude', PidTune, self.altitude_set_pid)
        # rospy.Subscriber('/pid_tuning_pitch', PidTune, self.pitch_set_pid)
        # rospy.Subscriber('/pid_tuning_yaw', PidTune, self.yaw_set_pid)


    # Imu callback function

    def imu_callback(self, msg):

        self.drone_orientation_quaternion[0] = msg.orientation.x
        self.drone_orientation_quaternion[1] = msg.orientation.y
        self.drone_orientation_quaternion[2] = msg.orientation.z
        self.drone_orientation_quaternion[3] = msg.orientation.w

    def drone_command_callback(self, msg):
        self.setpoint_cmd[0] = msg.rcRoll
        self.setpoint_cmd[2] = msg.rcYaw
        #print("Yaw here is ",self.setpoint_cmd[2])
        self.setpoint_cmd[1] = msg.rcPitch
        #self.setpoint_cmd[3] = msg.rcThrottle
        self.out_throttle = msg.rcThrottle
    # Callback function for /pid_tuning_roll
    # This function gets executed each time when /tune_pid publishes /pid_tuning_roll
    def roll_set_pid(self, roll):
        self.Kp[0] = roll.Kp * 0.06  # This is just for an example. You can change the ratio/fraction value accordingly
        self.Ki[0] = roll.Ki * 0.008
        self.Kd[0] = roll.Kd * 0.3
    
    def pitch_set_pid(self, pitch):
        self.Kp[1] = pitch.Kp * 0.06  # This is just for an example. You can change the ratio/fraction value accordingly
        self.Ki[1] = pitch.Ki * 0.008
        self.Kd[1] = pitch.Kd * 0.3
    
    def yaw_set_pid(self, yaw):
        self.Kp[2] = yaw.Kp * 1  # This is just for an example. You can change the ratio/fraction value accordingly
        self.Ki[2] = yaw.Ki * 0.08
        self.Kd[2] = yaw.Kd * 1.8
    
    def altitude_set_pid(self, altitude):
        self.Kp[3] = altitude.Kp * 0.06  # This is just for an example. You can change the ratio/fraction value accordingly
        self.Ki[3] = altitude.Ki * 0.000002
        self.Kd[3] = altitude.Kd * 0.3
       # print("booo")
    
    def gps_set_pid(self,msg):
    #   self.lat=msg.latitude
    #   self.lon=msg.longitude
        self.alt=msg.altitude
        #   print(self.alt)

    def pid(self):
        # Converting quaternion to euler angles
        (self.drone_orientation_euler[1], self.drone_orientation_euler[0], self.drone_orientation_euler[2]) = tf.transformations.euler_from_quaternion([self.drone_orientation_quaternion[0], self.drone_orientation_quaternion[1], self.drone_orientation_quaternion[2], self.drone_orientation_quaternion[3]])
       # print(self.drone_orientation_euler[0])
        self.drone_orientation_euler[0]=math.degrees(self.drone_orientation_euler[0])
        self.drone_orientation_euler[1]=math.degrees(self.drone_orientation_euler[1])
        self.drone_orientation_euler[2]=math.degrees(self.drone_orientation_euler[2])
      #  self.error[0] = 0 - self.drone_orientation_euler[0]
      #  print(self.error[0])
      #  self.pwm_pub.publish(self.pwm_cmd)
        
      #  Convertng the range from 1000 to 2000 in the range of -10 degree to 10 degree for roll axis
        self.setpoint_euler[0] = self.setpoint_cmd[0] * 0.02 - 30
        self.setpoint_euler[1] = self.setpoint_cmd[1] * 0.02 - 30
        self.setpoint_euler[2] = self.setpoint_cmd[2] * 0.02 - 30

        #self.setpoint_throttle = self.setpoint_cmd[3] * 1.024 - 1024
        #self.out_throttle = self.out_throttle * 1.024 - 1024

        self.error[0] = self.setpoint_euler[0] - self.drone_orientation_euler[0]
        self.error[1] = self.setpoint_euler[1] - self.drone_orientation_euler[1]
        self.error[2] = self.setpoint_euler[2] - self.drone_orientation_euler[2]
        #self.error[3] = self.fix_alt - self.alt

        #Compute all the working error variables
        self.errSum[0] = self.errSum[0] + (self.error[0] * self.sample_time)
        self.dErr[0] = (self.error[0] - self.prev_error[0]) / self.sample_time

        self.errSum[1] = self.errSum[1] + (self.error[1] * self.sample_time)
        self.dErr[1] = (self.error[1] - self.prev_error[1]) / self.sample_time

        self.errSum[2] = self.errSum[2] + (self.error[2] * self.sample_time)
        self.dErr[2] = (self.error[2] - self.prev_error[2]) / self.sample_time

        # self.errSum[3] = self.errSum[3] + (self.error[3] * self.sample_time)
        # self.dErr[3] = (self.error[3] - self.prev_error[3]) / self.sample_time

        #Compute PID Output
        self.out_roll = self.Kp[0] * self.error[0] + self.Ki[0] * self.errSum[0] + self.Kd[0] * self.dErr[0]
        
        self.out_pitch = self.Kp[1] * self.error[1] + self.Ki[1] * self.errSum[1] + self.Kd[1] * self.dErr[1]
  
        self.out_yaw = self.Kp[2] * self.error[2] + self.Ki[2] * self.errSum[2] + self.Kd[2] * self.dErr[2]

      #  if self.error[3] > 4:
      #      self.Ki[3] = 0
       # else:
       #     pass
        #self.out_throttle = self.Kp[3] * self.error[3] + self.Ki[3] * self.errSum[3] + self.Kd[3] * self.dErr[3]

        #Remember some variables for next time
        self.prev_error[0]= self.error[0]
        self.prev_error[1]= self.error[1]
        self.prev_error[2]= self.error[2]
        self.prev_error[3]= self.error[3]

        #-- self.setpoint_cmd[3]=self.out_throttle
        self.out_base =  400 + self.out_throttle*100
       # print("Hey",self.out_base)
        self.pwm_cmd.prop1 = self.out_base - self.out_roll + self.out_pitch - self.out_yaw 
        self.pwm_cmd.prop2 = self.out_base - self.out_roll - self.out_pitch + self.out_yaw 
        self.pwm_cmd.prop3 = self.out_base + self.out_roll - self.out_pitch - self.out_yaw 
        self.pwm_cmd.prop4 = self.out_base + self.out_roll + self.out_pitch + self.out_yaw 

        if self.pwm_cmd.prop1 > 1023:
            self.pwm_cmd.prop1 = 1023
        
        if self.pwm_cmd.prop1 < 0:
            self.pwm_cmd.prop1 = 0

        if self.pwm_cmd.prop2 > 1023:
            self.pwm_cmd.prop2 = 1023
        
        if self.pwm_cmd.prop2 < 0:
            self.pwm_cmd.prop2 = 0

        if self.pwm_cmd.prop3 > 1023:
            self.pwm_cmd.prop3 = 1023
        
        if self.pwm_cmd.prop3 < 0:
            self.pwm_cmd.prop3 = 0

        if self.pwm_cmd.prop4 > 1023:
            self.pwm_cmd.prop4 = 1023
        
        if self.pwm_cmd.prop4 < 0:
            self.pwm_cmd.prop4 = 0

      #  print(self.pwm_cmd.prop1)
       # print(self.pwm_cmd.prop2)
       # print(self.pwm_cmd.prop3)
       # print(self.pwm_cmd.prop4)
        
        self.pwm_pub.publish(self.pwm_cmd)
      #  self.roll_pub.publish(self.error[0])
      #  self.pitch_pub.publish(self.error[1])
        #self.alt_pub.publish(self.error[3])
        #print(self.error[3])
        
if __name__ == '__main__':

    e_drone = Edrone()
    r = rospy.Rate(e_drone.sample_time)  # specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
    while not rospy.is_shutdown():
        e_drone.pid()
        r.sleep()

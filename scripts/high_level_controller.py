#!/usr/bin/env python3
# *******************************************************
# Type: High level mouse controller from ROS
# 
# High level controller for the mouse which then passes 
# high level gait changes and commands to the motion module
# 
# Author: Alex Rohregger
# Contact: alex.rohregger@tum.de
# Last-edited: 26.05.2021
# *********************************************************

# Import the leg and motionplanner modules
from time import sleep

# Import other relevant libraries for ROS
import os
import rospkg
import numpy as np
import sys

from time import sleep

rp = rospkg.RosPack()
script_path = os.path.join(rp.get_path("mouse_controller"), "models")
sys.path.append(script_path)

from numpy.core.numerictypes import maximum_sctype
import rospy
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Float32
from mouse_controller.msg import Floats, desired_cmd, mouse_sensors


class High_Level_Control:

    def __init__(self):
        self.sensors_imu = [0.0]*6
        self.main()

    def callback_mouse_sensors(self, data):
        # In here we can set the measured values relevant for our controller
        # Mainly the following:
        # Servo of legs: data.servo_pos_leg
        # Servo of aux: data.servo_pos_aux
        # contact sensors: data.contact_sensors
        # imu (not relevant but for reference): data.imu
        self.sensors_imu = data.imu

    def main_hl(self,rate):
        # main starter method
        # print("Starting mouse node")
        dry = 0

        # Initialize the node
        rospy.init_node("high_level_controller", anonymous=True)
        rospy.Subscriber("sensors_mouse", mouse_sensors, self.callback_mouse_sensors, queue_size=1)
        self.pub_control = rospy.Publisher('desired_cmd', desired_cmd, queue_size=1)
        r = rospy.Rate(rate)

        while(not rospy.is_shutdown()):
            vel_in = 0.3*rospy.get_param("/vel_ly")
            turn_rate = rospy.get_param("/vel_rx")
            vel_d, tr_d = self.high_level_control(vel_in, turn_rate, mode=False)
            self.gen_control_message(vel_d, tr_d)
            r.sleep()

    def high_level_control(self, vel_in: float, turn_rate: float, mode: bool):
        # High level control function
        # If "mode" = false, then just pass the values directly
        if mode:
            return (0.0, 0.0)
        else:
            return (vel_in, turn_rate)

    def gen_control_message(self,vel_d: float, tr_d: float):
        # Generate the necessary ROS custom message
        desired_cmds = desired_cmd()
        desired_cmds.vel = vel_d
        desired_cmds.turn_rate = tr_d
        self.pub_control.publish(desired_cmds)
    
    def main(self):
        # Length of pre-run and then data collection
        rate = 100

        try: 
            self.main_hl(rate)
        except rospy.ROSInterruptException:
            pass    

if __name__ == "__main__":
    High_Level_Control()

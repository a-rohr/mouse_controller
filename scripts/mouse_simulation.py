#!/usr/bin/env python3

from mujoco_py import load_model_from_path, MjSim, MjViewer
import os
import rospkg
import math
import numpy as np
import random
import sys

rp = rospkg.RosPack()
script_path = os.path.join(rp.get_path("mouse_controller"), "models")
sys.path.append(script_path)

from numpy.core.numerictypes import maximum_sctype
import pandas as pd
import time
import pathlib
import rospy
import roslib
from std_msgs.msg import Empty, _String

# *******************************************************
# Type: Test Script
# 
# MuJoCo test of a four legged walker
# This test is on a dynamic four legged walker model
# to observe the initial gait performance.
# 
# Author: Alex Rohregger
# Contact: alex.rohregger@tum.de
# Last-edited: 26.02.2021
# *********************************************************

# Import the leg and motionplanner modules
from mouse_controller.leg_unit_class import Leg_Unit
from mouse_controller.four_legged_body import Quadruped_Walker

from std_msgs.msg import Empty, _String
from rospy.numpy_msg import numpy_msg
from mouse_controller.msg import Floats, mouse_sensors

import os


class Mouse_SIM:

    def __init__(self):
        self.init_variables()
        self.main()

    def init_variables(self):
        # Import model relevant parameters
        # load up of the model of front leg type 1
        self.model_name = "dynamic_4l_t3.xml"
        self.model_path = os.path.join(rp.get_path("mouse_controller"), "models", self.model_name)
        self.model = load_model_from_path(self.model_path)
        # model = load_model_from_path("../models/realistic_leg_1_jumper.xml")
        self.sim = MjSim(self.model)

        # initialize viewer for rendering
        # viewer = MjViewer(sim)


        # Number of test runs to get the kinematics
        self.test_runs = 100000
        self.dead_time = 2000


        # Allowed range of the motors
        self.m1_range = {"max": 1.0,
                    "min": -1.57}
        self.m2_range = {"max": 1.4,
                    "min": -2.0}

        self.sim_state = self.sim.get_state()

        self.sim.set_state(self.sim_state)

        # initialize viewer for rendering
        self.viewer = MjViewer(self.sim)
        self.q_values = np.array([0]*12)


    def callback_q_values(self,data):
        # Catch values for the q-values from ROS
        self.q_values = np.array((data.data))
        # q_spine = rospy.get_param("/vel_rx")
        # q_aux = np.array(([-q_spine,0,0,q_spine]))
        # q_values = np.concatenate((q_values_leg, q_aux))
        print("Control values sent: {}".format(self.q_values))
        # sim.step()
        # viewer.render()
        

    def run_simulation(self, rate):
        # main starter method
        # print("Starting mouse node")
        dry = 0

        # Initialize the node
        rospy.init_node("mouse_simulation", anonymous=True)
        r = rospy.Rate(rate)

        # Subscribe to the q_values of the leg controller
        rospy.Subscriber('q_values', Floats, self.callback_q_values, queue_size = 1)
        self.pub_sensor = rospy.Publisher('sensors_mouse', mouse_sensors, queue_size=1)

        while(not rospy.is_shutdown()):
            self.pub_sensor.publish(self.gen_sensor_message(self.sim.data.sensordata))
            self.sim.data.ctrl[:] = self.q_values
            self.sim.step()
            self.viewer.render()
            r.sleep()

    def gen_sensor_message(self,data):
        mouse_sensor = mouse_sensors()
        mouse_sensor.servo_pos_leg = data[:8]
        mouse_sensor.servo_pos_aux = data[8:12]
        mouse_sensor.contact_sensors = data[12:16]
        mouse_sensor.imu_sensor = data[16:]
        return mouse_sensor

    def idle_motion(self):
        for i in range(self.dead_time):
            self.sim.data.ctrl[:] = self.q_values
            self.sim.step()
            current_servo_values = self.sim.data.qpos[:]
            print(current_servo_values)
            self.viewer.render() 

    def main(self):
        # Length of pre-run and then data collection
        self.idle_motion()
        rate = 200

        try: 
            self.run_simulation(rate)
        except rospy.ROSInterruptException:
            pass    

if __name__ == "__main__":
    print("Let's start")
    Mouse_SIM()


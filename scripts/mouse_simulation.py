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
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from mouse_controller.msg import Floats
from sensor_msgs.msg import Joy 
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import InteractiveMarkerInit

import os

# Import model relevant parameters
# load up of the model of front leg type 1
model_name = "dynamic_4l_t3.xml"
model_path = os.path.join(rp.get_path("mouse_controller"), "models", model_name)
model = load_model_from_path(model_path)
# model = load_model_from_path("../models/realistic_leg_1_jumper.xml")
sim = MjSim(model)

# initialize viewer for rendering
# viewer = MjViewer(sim)


# Number of test runs to get the kinematics
test_runs = 100000
dead_time = 2000


# Allowed range of the motors
m1_range = {"max": 1.0,
            "min": -1.57}
m2_range = {"max": 1.4,
            "min": -2.0}

sim_state = sim.get_state()

sim.set_state(sim_state)

# initialize viewer for rendering
viewer = MjViewer(sim)

def callback_q_values(data):
    # Reading out the sensor data
    csv_fl = sim.data.sensordata[:]
    csv_fr = sim.data.sensordata[4:]
    csv_rl = sim.data.sensordata[8:]
    csv_rr = sim.data.sensordata[12:]

    # Catch values for the q-values from ROS
    print("Callback hit")
    print(data.data)
    q_values = np.array((data.data))
    # q_spine = rospy.get_param("/vel_rx")
    # q_aux = np.array(([-q_spine,0,0,q_spine]))
    # q_values = np.concatenate((q_values_leg, q_aux))
    print("Control values sent: {}".format(q_values))
    print(q_values.shape)
    sim.data.ctrl[:] = q_values
    # sim.step()
    # viewer.render()
    

def run_simulation(rate):
    # main starter method
    # print("Starting mouse node")
    dry = 0

    # Initialize the node
    rospy.init_node("mouse_simulation", anonymous=True)
    r = rospy.Rate(rate)

    # Subscribe to the q_values of the leg controller
    rospy.Subscriber('q_values', Floats, callback_q_values, queue_size = 1)

    while(not rospy.is_shutdown()):
        sim.step()
        viewer.render()
        r.sleep()

def idle_motion(pos):
    for i in range(dead_time):
        sim.data.ctrl[:] = pos
        sim.step()
        current_servo_values = sim.data.qpos[:]
        print(current_servo_values)
        viewer.render() 

def main():
    # Length of pre-run and then data collection
    idle_motion(np.array([0]*12))
    rate = 200

    try: 
        run_simulation(rate)
    except rospy.ROSInterruptException:
        pass    


if __name__ == "__main__":
    
    main()


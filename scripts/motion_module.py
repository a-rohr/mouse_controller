#!/usr/bin/env python3

from mujoco_py import load_model_from_path, MjSim, MjViewer
import os
import rospkg
import math
import numpy as np
import random
import sys

from time import sleep

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
from mouse_controller.leg_controller import Leg_Controller
from mouse_controller.state_machine.leg_state_machine import Leg_State_Machine
from mouse_controller.mouse_parameters_dir import Gait_Parameters, Mouse_Parameters
from time import sleep



from std_msgs.msg import Empty, _String
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from mouse_controller.msg import Floats
from sensor_msgs.msg import Joy 
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import InteractiveMarkerInit

import os

def motion_node(rate):
    # main starter method
    # print("Starting mouse node")
    dry = 0

    # Initialize the node
    rospy.init_node("mouse_node_test", anonymous=True)
    pub_x = rospy.Publisher('leg_outputs_x', Floats, queue_size=1)
    pub_y = rospy.Publisher('leg_outputs_y', Floats, queue_size=1)
    pub_vel = rospy.Publisher('velocity_joy', Float32, queue_size=1)
    pub_turn = rospy.Publisher('turnrate_joy', Float32, queue_size=1)
    pub_q_values = rospy.Publisher('q_values', Floats, queue_size=1)
    r = rospy.Rate(rate)

    gait_parameters2 = Gait_Parameters()
    mouse_parameters = Mouse_Parameters()
    general_st_parameters2 = gait_parameters2.st_trot_parameters
    front_leg_parameters2 = gait_parameters2.st_trot_param_f
    rear_leg_parameters2 = gait_parameters2.st_trot_param_r

    # Initialize the key components of the motion module
    fsm = Leg_State_Machine(general_st_parameters2)
    leg_controller = Leg_Controller(gait_parameters2, mouse_parameters)
    fsm.timer.reset_times()
    sleep(0.002)

    # Subscribe to the ps4 controller
    # rospy.Subscriber("joy", Joy, callback_vel, queue_size=1)
    target_leg_x_f = Floats()
    target_leg_y_f = Floats()
    target_q_values = Floats()

    while(not rospy.is_shutdown()):
        vel_in = 0.5*rospy.get_param("/vel_ly")
        turn_rate = rospy.get_param("/vel_rx")

        vel = vel_in * np.ones((4,))

        # Steps of the full controller to generate values
        leg_states, leg_timings, norm_time = fsm.run_state_machine()
        target_leg_positions, q_legs, q_spine = leg_controller.run_controller(leg_states, leg_timings, norm_time, vel, turn_rate)
        target_leg_positions.astype(dtype=np.float32)
        q_spine = (0.4*turn_rate + (1-np.abs(turn_rate))*q_spine)
        q_values = np.concatenate((q_legs,np.array(([0,0,0,q_spine]))))
        q_values.astype(dtype=np.float32)
        print(q_values)

        # This step handles the publishing to the ROS backend
        target_leg_x = target_leg_positions[:,0]
        target_leg_y = target_leg_positions[:,1]
        target_leg_x_f.data = target_leg_x.tolist()
        target_leg_y_f.data = target_leg_y.tolist()
        target_q_values.data = q_values.tolist()
        pub_x.publish(target_leg_x_f)
        pub_y.publish(target_leg_y_f)
        pub_vel.publish(vel_in)
        pub_turn.publish(turn_rate)
        pub_q_values.publish(target_q_values)

        r.sleep()


if __name__ == "__main__":
    try:

        motion_node(rate=100)
    except rospy.ROSInterruptException:
        pass

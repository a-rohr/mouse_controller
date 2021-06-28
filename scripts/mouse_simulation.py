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

    print(data)

    q_values = np.concatenate((q_values_fl, q_values_fr,q_values_rl,q_values_rr, max_vel_s))
    print("Control values sent: {}".format(q_values))
    print(q_values.shape)
    sim.data.ctrl[:] = q_values
    sim.step()
    viewer.render()
    

def run_simulation(rate):
    # main starter method
    # print("Starting mouse node")
    dry = 0

    # Initialize the node
    rospy.init_node("mouse_simulation", anonymous=True)
    pub_x = rospy.Publisher('leg_outputs_x', Floats, queue_size=1)
    pub_y = rospy.Publisher('leg_outputs_y', Floats, queue_size=1)
    pub_vel = rospy.Publisher('velocity_joy', Float32, queue_size=1)
    r = rospy.Rate(rate)

    # Subscribe to the ps4 controller
    # rospy.Subscriber("joy", Joy, callback_vel, queue_size=1)
    target_leg_x_f = Floats()
    target_leg_y_f = Floats()

    while(not rospy.is_shutdown()):
        vel_in = 0.05*rospy.get_param("/vel_ly")
        vel = vel_in * np.ones((4,))
        leg_states, leg_timings = fsm.run_state_machine()
        target_leg_positions, q_values = leg_controller.run_controller(leg_states, leg_timings, vel)
        target_leg_positions.astype(dtype=np.float32)
        print(q_values)
        target_leg_x = target_leg_positions[:,0]
        target_leg_y = target_leg_positions[:,1]
        target_leg_x_f.data = target_leg_x.tolist()
        target_leg_y_f.data = target_leg_y.tolist()
        pub_x.publish(target_leg_x_f)
        pub_y.publish(target_leg_y_f)
        pub_vel.publish(vel_in)
        r.sleep()

def leg_movement2(mouse, max_vel, trajectory_r, max_step_r, trajectory_f, max_step_f, trajectory_rf, trajectory_rr):
    # This method runs a full leg test for given trajectory
    # INPUT: 
    #       none
    # OUTPUT: 
    #       none

    lu_fl = mouse["fl"]
    lu_fr = mouse["fr"]
    lu_rl = mouse["rl"]
    lu_rr = mouse["rr"]

    lu_fl.new_trajectory(trajectory_f)
    lu_rr.new_trajectory(trajectory_r)
    set_to_zero = False

    current_servo_values = 0
    count = 0
    leg_value = 0
    # Running through the full trajectory until the MP has no further trajectory point
    while(True):
        max_vel_f = 1.4*(rospy.get_param("/vel_ly"))
        max_vel_s = np.array([0,0,rospy.get_param("/vel_rx")])

        if max_vel_f <= 0.0:
            lu_fr.mp_leg.max_v = 0.01
            lu_fl.mp_leg.max_v = 0.01
            lu_rr.mp_leg.max_v = 0.01
            lu_rl.mp_leg.max_v = 0.01
            lu_fr.new_trajectory(trajectory_rf)
            lu_fl.new_trajectory(trajectory_rf)
            lu_rr.new_trajectory(trajectory_rr)
            lu_rl.new_trajectory(trajectory_rr)
            set_to_zero = True
        else:
            if set_to_zero:
                lu_fl.new_trajectory(trajectory_f)
                lu_rr.new_trajectory(trajectory_r)
                leg_value = 0
                set_to_zero = False

            lu_fr.mp_leg.max_v = max_vel_f
            lu_fl.mp_leg.max_v = max_vel_f
            lu_rr.mp_leg.max_v = max_vel_f
            lu_rl.mp_leg.max_v = max_vel_f

            if lu_fl.mp_status() <= 1 and lu_rr.mp_status() <= 1 and leg_value == 0:
                lu_fr.new_trajectory(trajectory_f)
                lu_rl.new_trajectory(trajectory_r)
                leg_value = 1
            elif lu_rl.mp_status() <= 1 and lu_fr.mp_status() <= 1 and leg_value == 1:
                lu_fl.new_trajectory(trajectory_f)
                lu_rr.new_trajectory(trajectory_r)
                leg_value = 0

        print(sim.data.sensordata)

        csv_fl = sim.data.sensordata[:]
        csv_fr = sim.data.sensordata[4:]
        csv_rl = sim.data.sensordata[8:]
        csv_rr = sim.data.sensordata[12:]

        time_start = time.perf_counter()
        
        q_values_fl = lu_fl.kinematic_update(csv_fl,max_step_f)
        q_values_fr = lu_fr.kinematic_update(csv_fr,max_step_f)
        # q_values_fl = np.array([0,0])
        # q_values_fr = np.array([0,0])
        q_values_rl = lu_rl.kinematic_update(csv_rl,max_step_r)
        q_values_rr = lu_rr.kinematic_update(csv_rr,max_step_r)
        # print("Elapsed total sim time: {}".format(time.perf_counter() - time_start))

        q_values = np.concatenate((q_values_fl, q_values_fr,q_values_rl,q_values_rr, max_vel_s))
        print("Control values sent: {}".format(q_values))
        print(q_values.shape)
        sim.data.ctrl[:] = q_values
        sim.step()
        viewer.render()
        count += 1


def idle_motion(pos):
    for i in range(dead_time):
        sim.data.ctrl[:] = pos
        sim.step()
        current_servo_values = sim.data.qpos[:]
        print(current_servo_values)
        viewer.render() 


def main():
    # Length of pre-run and then data collection
    idle_motion(np.array([0]*11))

    while(True):
        trotting_test(mouse)
        #full_body_motion()
        # walking_test(mouse)
        
        # standing_leg_test(mouse)
        # swaying_leg_test(mouse)
    



if __name__ == "__main__":
    
    main()


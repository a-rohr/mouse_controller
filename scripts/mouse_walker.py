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


def walking_test(mouse):
    # Trajectory Setup
    max_vel = 0.01

    # *** Rear Leg Gait Pattern ***
    n_r = np.array([0, -0.045])

    st_r = np.array([0.04, -0.023])
    a_r = np.array([0.04, -0.055])
    b_r = np.array([-0.015, -0.04])

    s_r = 6

    trajectory_r = [st_r,b_r,a_r]
    max_step_r = [0.02*s_r, 0.02*s_r, 0.02*s_r]

    # *** Front Leg Gait Pattern ***
    n_f = np.array([0, -0.045])

    st_f = np.array([-0.035, -0.025])
    a_f = np.array([0.005, -0.02])
    b_f = np.array([0.005, -0.05])

    s_f = 6

    trajectory_f = [a_f,st_f, b_f]
    max_step_f = [0.02*s_f,0.02*s_f,0.02*s_f]

    trajectory_rest = [n_f,n_f,n_f]

    leg_movement2(mouse, max_vel, trajectory_r, max_step_r, trajectory_f, max_step_f, trajectory_rest)

def trotting_test(mouse):
    # Trotting cycle trial of walking
    # Trajectory Setup
    # max_vel = 0.01
        # Trajectory Setup
    max_vel = 0.4

    # # *** Rear Leg Gait Pattern ***
    st_r = np.array([0.00, -0.04])
    a_r = np.array([0.03, -0.05])
    b_r = np.array([0.0125, -0.04])
    c_r = np.array([-0.0075, -0.05])
    d_r = np.array([0.0125, -0.06])

    trajectory_r = [a_r, b_r, c_r, d_r]

    s_r = 7

    max_step_r = [0.02*s_r] * len(trajectory_r)

    # *** Front Leg Gait Pattern ***
    st_f = np.array([0.00, -0.035])
    a_f = np.array([0.015, -0.035])
    b_f = np.array([-0.005, -0.025])
    c_f = np.array([-0.020, -0.035])
    d_f = np.array([-0.0025, -0.045])

    trajectory_f = [a_f, b_f, c_f, d_f]

    s_f = 7

    max_step_f = [0.02*s_f] * len(trajectory_f)

    # *** Rear Leg Gait Pattern ***
    n_r = np.array([0.03, -0.04])

    a_r = np.array([0.03, -0.03])  
    c_r = np.array([0.03, -0.05])

    s_r = 10

    # trajectory_r = [a_r, c_r]
    # max_step_r = [0.02*s_r, 0.02*s_r]

    # *** Front Leg Gait Pattern ***
    n_f = np.array([0, -0.035])

    a_f = np.array([0, -0.025])  
    c_f = np.array([0, -0.045])

    s_f = 10

    # trajectory_f = [a_f,c_f]
    # max_step_f = [0.02*s_f,0.02*s_f]

    trajectory_rf = [n_f,n_f]
    trajectory_rr = [n_r,n_r]

    leg_movement2(mouse, max_vel, trajectory_r, max_step_r, trajectory_f, max_step_f, trajectory_rf, trajectory_rr)

def leg_movement(mouse, max_vel, trajectory_r, max_step_r, trajectory_f, max_step_f):
    # This method runs a full leg test for given trajectory
    # INPUT: 
    #       none
    # OUTPUT: 
    #       none

    # Define the necessary leg parameters for front legs
    fr_t1_param = {'lr0':0.033, 'rp': 0.008, 'd1': 0.0128, 'l1': 0.025, 
                    'l2': 0.018, 'l3': 0.028, 'l4': 0.015,'theta3':np.pi/10}

    lu_fl = mouse["fl"]
    lu_fr = mouse["fr"]

    # Define the necessary leg parameters for rear legs
    rr_t3_param = {'lr0':0.02678, 'rp': 0.008, 'd1': 0.0128, 'l1': 0.03, 
                    'l2': 0.0084, 'l3': 0.045, 'l4': 0.025,'theta3':110*np.pi/180}

    lu_rl = mouse["rl"]
    lu_rr = mouse["rr"]


    lu_fl.new_trajectory(trajectory_f)
    lu_fr.new_trajectory(trajectory_f)
    lu_rl.new_trajectory(trajectory_r)
    lu_rr.new_trajectory(trajectory_r)

    # idle_motion(np.array([0]*8))

    current_servo_values = 0
    count = 0
    # Running through the full trajectory until the MP has no further trajectory point
    while(count < 2000):

        if lu_fl.mp_status() == -1 and lu_fr.mp_status() == -1 and lu_rl.mp_status() == -1 and lu_rr.mp_status() == -1:
            lu_fl.new_trajectory(trajectory_f)
            lu_fr.new_trajectory(trajectory_f)
            lu_rl.new_trajectory(trajectory_r)
            lu_rr.new_trajectory(trajectory_r)

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
        print("Elapsed total sim time: {}".format(time.perf_counter() - time_start))

        q_values = np.concatenate((q_values_fl, q_values_fr,q_values_rl,q_values_rr))
        print("Control values sent: {}".format(q_values))
        sim.data.ctrl[:] = q_values
        sim.step()
        viewer.render()
        count += 1

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

def leg_movement3(mouse, max_vel, trajectory_r, max_step_r, trajectory_f, max_step_f):
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

    current_servo_values = 0
    count = 0
    leg_value = 0
    # Running through the full trajectory until the MP has no further trajectory point
    while(count < 2000):

        if lu_fl.mp_status() <= 1 and leg_value == 0:
            lu_rl.new_trajectory(trajectory_r)
            leg_value = 1
        elif lu_rl.mp_status() <= 1 and leg_value == 1:
            lu_fr.new_trajectory(trajectory_f)
            leg_value = 2
        elif lu_fr.mp_status() <= 1 and leg_value == 2:
            lu_rr.new_trajectory(trajectory_r)
            leg_value = 3
        elif lu_rr.mp_status() <= 1 and leg_value == 3:
            lu_fl.new_trajectory(trajectory_f)
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
        print("Elapsed total sim time: {}".format(time.perf_counter() - time_start))

        q_values = np.concatenate((q_values_fl, q_values_fr,q_values_rl,q_values_rr))
        print("Control values sent: {}".format(q_values))
        sim.data.ctrl[:] = q_values
        sim.step()
        viewer.render()
        count += 1

def full_body_motion():
    # New body motion system for whole body motion

    mouse = Quadruped_Walker()
    n_f = np.array([0, -0.045])
    trajectory_r = [n_f,n_f,n_f]
    timestep_r = [0.1, 0.1, 0.1]

    while(True):
        vel_x = (rospy.get_param("/vel_rx"))
        vel_y = -0.02*(rospy.get_param("/vel_lx"))
        vel_z = 0.02*(rospy.get_param("/vel_ly"))

        rot = np.array([[0, 0, 0],
                        [0, 0, 0],
                        [0, 0, 0]])

        vel = np.array([[vel_x],
                        [vel_y],
                        [vel_z]])

        current_servos = sim.data.sensordata
        if(vel.sum() != 0):
            q_vals = mouse.top_level_kinematics(vel,rot,current_servos)
        else:
            q_vals = mouse.trajectory_level_kinematics(trajectory_r,current_servos,timestep_r)
            
        sim.data.ctrl[:] = q_vals
        sim.step()
        viewer.render()


def idle_motion(pos):
    for i in range(dead_time):
        sim.data.ctrl[:] = pos
        sim.step()
        current_servo_values = sim.data.qpos[:]
        print(current_servo_values)
        viewer.render() 


def main():
    # Length of pre-run and then data collection
    max_vel = 0.1
    # Define the necessary leg parameters for front legs
    fr_t1_param = {'lr0':0.033, 'rp': 0.008, 'd1': 0.0128, 'l1': 0.0295, 
                    'l2': 0.0145, 'l3': 0.0225, 'l4': 0.0145,'theta3':23*np.pi/180}

    lu_fl = Leg_Unit('fr3',fr_t1_param,max_vel,threshold=0.0005)
    lu_fr = Leg_Unit('fr3',fr_t1_param,max_vel,threshold=0.0005)

    # Define the necessary leg parameters for rear legs
    rr_t3_param = {'lr0':0.02678, 'rp': 0.008, 'd1': 0.0128, 'l1': 0.0317, 
                    'l2': 0.02, 'l3': 0.0305, 'l4': 0.0205,'theta3':73*np.pi/180}

    lu_rl = Leg_Unit('rr3',rr_t3_param,max_vel,threshold=0.0005)
    lu_rr = Leg_Unit('rr3',rr_t3_param,max_vel,threshold=0.0005)

    mouse = {"fl": lu_fl,
             "fr": lu_fr,
             "rl": lu_rl,
             "rr": lu_rr}
    idle_motion(np.array([0]*11))

    while(True):
        trotting_test(mouse)
        #full_body_motion()
        # walking_test(mouse)
        
        # standing_leg_test(mouse)
        # swaying_leg_test(mouse)
    



if __name__ == "__main__":
    
    main()


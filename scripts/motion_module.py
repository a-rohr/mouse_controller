#!/usr/bin/env python3
# *******************************************************
# Type: Motion controller
# 
# Motion controller for the mouse
# Handles state machine and low level spine and leg control.
# 
# Author: Alex Rohregger
# Contact: alex.rohregger@tum.de
# Last-edited: 26.04.2021
# *********************************************************

# Import the leg and motionplanner modules
from mouse_controller.leg_controller import Leg_Controller
from mouse_controller.state_machine.leg_state_machine import Leg_State_Machine
from mouse_controller.mouse_parameters_dir import Gait_Parameters, Mouse_Parameters
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

class Motion_Module:
    def __init__(self):
        self.init_ros_variables()
        self.init_mouse_variables()
        self.init_controllers()
        self.main()

    def init_ros_variables(self):
        self.pub_x = rospy.Publisher('leg_outputs_x', Floats, queue_size=1)
        self.pub_y = rospy.Publisher('leg_outputs_y', Floats, queue_size=1)
        self.pub_vel = rospy.Publisher('velocity_joy', Float32, queue_size=1)
        self.pub_turn = rospy.Publisher('turnrate_joy', Float32, queue_size=1)
        self.pub_q_values = rospy.Publisher('q_values', Floats, queue_size=1)
        self.target_leg_x_f = Floats()
        self.target_leg_y_f = Floats()
        self.target_q_values = Floats()
    
    def init_mouse_variables(self):
        self.gait_parameters2 = Gait_Parameters()
        self.mouse_parameters = Mouse_Parameters()
        self.general_st_parameters2 = self.gait_parameters2.st_trot_parameters
        self.front_leg_parameters2 = self.gait_parameters2.st_trot_param_f
        self.rear_leg_parameters2 = self.gait_parameters2.st_trot_param_r

    def init_controllers(self):
        # Initialize the key components of the motion module
        # Spine modes:
        # 0: purely turning motion, nothing else
        # 1: turning motion + spine modulation
        # 2: turning motion + balance mode (balance mode for 2 and 3 leg contact)
        self.spine_mode = 0
        self.offset_mode = False
        self.balance_mode = False
        self.fsm = Leg_State_Machine(self.general_st_parameters2)
        self.leg_controller = Leg_Controller(self.gait_parameters2, self.mouse_parameters)
        self.vel_in = 0.0
        self.turn_rate = 0.0
        self.sensors_leg_servo = [0.0]*8
        self.sensors_aux_servo = [0.0]*4
        self.sensors_contact = [0.0]*4
        self.buttons = [0]*4
        self.prev_buttons = [0]*4
        self.leg_n = 0

    def callback_desired_cmd(self, data):
        # Callback of subscriber to high level desired cmd message
        # Contains two points: vel: float32 || turn_rate: float32
        self.vel_in = data.vel
        self.turn_rate = data.turn_rate
        self.prev_buttons = self.buttons
        self.buttons = data.buttons

    def callback_mouse_sensors(self, data):
        # In here we can set the measured values relevant for our controller
        # Mainly the following:
        # Servo of legs: data.servo_pos_leg
        # Servo of aux: data.servo_pos_aux
        # contact sensors: data.contact_sensors
        # imu (not relevant but for reference): data.imu
        self.sensors_leg_servo = data.servo_pos_leg
        self.sensors_aux_servo = data.servo_pos_aux
        self.sensors_contact = data.servo_pos_leg

    def motion_node(self,rate):
        # main starter method
        # print("Starting mouse node")
        dry = 0

        # Initialize the node
        rospy.init_node("motion_module", anonymous=True)
        r = rospy.Rate(rate)

        self.fsm.timer.reset_times()
        sleep(0.002)

        rospy.Subscriber("desired_cmd", desired_cmd, self.callback_desired_cmd, queue_size=1)
        rospy.Subscriber("sensors_mouse", mouse_sensors, self.callback_mouse_sensors, queue_size=1)

        # Subscribe to the ps4 controller
        # rospy.Subscriber("joy", Joy, callback_vel, queue_size=1)

        while(not rospy.is_shutdown()):
            # self.vel_in = 0.3*rospy.get_param("/vel_ly")
            # self.turn_rate = rospy.get_param("/vel_rx")
            self.check_button_sets()
            vel = self.vel_in * np.ones((4,))
            leg_states, leg_timings, norm_time = self.fsm.run_state_machine()
            if not self.balance_mode:
                # Steps of the full controller to generate values
                target_leg_positions, q_legs, q_spine = self.leg_controller.run_controller(leg_states, leg_timings, norm_time, vel, self.turn_rate, self.spine_mode, self.offset_mode)
            else:
                target_leg_positions, q_legs, q_spine = self.leg_balance_tester(leg_timings, norm_time, vel, self.turn_rate, self.spine_mode, self.offset_mode)
            self.gen_messages(target_leg_positions, q_legs, q_spine=0)
            r.sleep()

    def check_button_sets(self):
        if self.prev_buttons[0] != self.buttons[0] and self.prev_buttons[0] == 0:
            self.leg_n = (self.leg_n+1)%4
        if self.prev_buttons[1] != self.buttons[1] and self.prev_buttons[1] == 0:
            if self.offset_mode:
                self.offset_mode = False
            else:
                self.offset_mode = True
        if self.prev_buttons[2] != self.buttons[2] and self.prev_buttons[2] == 0:
            self.spine_mode = (self.spine_mode+1)%3
        if self.prev_buttons[3] != self.buttons[3] and self.prev_buttons[3] == 0:
            if self.offset_mode:
                self.balance_mode = False
            else:
                self.balance_mode = True
    
    def leg_balance_tester(self, leg_timings, norm_time, vel, turn_rate, spine_mode, offset_mode):
        # Mode to test the balance ability of the spine balance mode
        
        leg_states = np.array([1,1,1,1])
        leg_states[self.leg_n] = 0
        turn_rate = 0
        tl, ql, q_spine = self.leg_controller.run_controller(leg_states, leg_timings, norm_time, vel, turn_rate, spine_mode, offset_mode)
        q1_c = -0.15 + 3*self.vel_in
        q2_c = -0.66 + self.turn_rate
        q_legs = np.array([0.42,0.6,0.42,0.6,-0.15,-0.66,q1_c,q2_c])
        q_legs = self.balance_mode_gen_q_leg(ql,leg_states)
        target_leg_positions = np.ones((4,2))
        # q_spine = 0.0
        return (target_leg_positions, q_legs, q_spine)

    def balance_mode_gen_q_leg(self, ql, leg_states):
        ad_val = np.abs(leg_states - np.ones((4,)))
        ad_val = np.reshape(ad_val,(4,1))
        front_neutral = np.array([0.42,0.6])
        rear_neutral = np.array([-0.15,-0.66])

        static_vals = np.array([front_neutral,
                                front_neutral,
                                rear_neutral,
                                rear_neutral])

        # static_vals = np.reshape(ql, (4,2))
        
        inputs = np.array([3*self.vel_in, self.turn_rate])
        inputs = np.reshape(inputs,(1,2))

        static_vals = static_vals + np.dot(ad_val,inputs)
        q_legs = np.reshape(static_vals,(8,))
        return q_legs
    

    def gen_messages(self, target_leg_positions, q_legs, q_spine):
        target_leg_positions.astype(dtype=np.float32)
        q_values = np.concatenate((q_legs,np.array(([0,0,0,q_spine]))))
        q_values.astype(dtype=np.float32)
        # print(q_values)

        # This step handles the publishing to the ROS backend
        target_leg_x = target_leg_positions[:,0]
        target_leg_y = target_leg_positions[:,1]
        self.target_leg_x_f.data = target_leg_x.tolist()
        self.target_leg_y_f.data = target_leg_y.tolist()
        self.target_q_values.data = q_values.tolist()
        self.pub_x.publish(self.target_leg_x_f)
        self.pub_y.publish(self.target_leg_y_f)
        self.pub_vel.publish(self.vel_in)
        self.pub_turn.publish(self.turn_rate)
        self.pub_q_values.publish(self.target_q_values)

    def main(self):
        try:

            self.motion_node(rate=100)
        except rospy.ROSInterruptException:
            pass

if __name__ == "__main__":
    Motion_Module()

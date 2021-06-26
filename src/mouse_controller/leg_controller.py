import numpy as np
import rospkg

from mouse_controller.trajectory_generator.trajectory_generator import Leg_Trajectory_Generator

from mouse_controller.leg_unit_class import Leg_Unit
from mouse_controller.mouse_parameters_dir import Gait_Parameters, Mouse_Parameters

class Leg_Controller:

    def __init__(self, gait_parameters):
        self.gait_parameters = gait_parameters
        self.ik_legs = Inverse_Leg_Kinematics()
        self.setup_trajectory_generators()
        self.previous_leg_states = -1*np.ones((4,),dtype = int)
        print("Initialized leg controller")

    def setup_trajectory_generators(self):
        front_leg_parameters = self.gait_parameters.st_trot_param_f
        rear_leg_parameters = self.gait_parameters.st_trot_param_r

        fl_traj = Leg_Trajectory_Generator(front_leg_parameters,0)
        fr_traj = Leg_Trajectory_Generator(front_leg_parameters,1)
        rl_traj = Leg_Trajectory_Generator(rear_leg_parameters,2)
        rr_traj = Leg_Trajectory_Generator(rear_leg_parameters,3)

        self.traj_obj = {0: fl_traj,
                         1: fr_traj,
                         2: rl_traj,
                         3: rr_traj}

    def update_leg_traj_param(self, gait_param):
        #Empty for now
        return

    def run_controller(self, leg_states, leg_timings, leg_velocities, turn_rate = 0):
        alphas = self.compute_turn_alphas(turn_rate)

        next_leg_positions = self.compute_next_leg_positions(leg_states, leg_timings, leg_velocities, alphas)

        # Only for testing purposes
        return next_leg_positions
        leg_q_values = self.ik_legs.run_inverse_leg_kinematics(next_leg_positions)

        return leg_q_values 

    def compute_turn_alphas(self, turn_rate):
        # In here we compute alpha value adjustments for specified turn rates
        # TO-DO

        return np.ones((4,))

    def compute_new_trajectory(self, leg_velocities, turn_rates):
        return

    def compute_next_leg_positions(self, leg_states, leg_timings, leg_velocities, alphas):
        status_change = np.abs(leg_states-self.previous_leg_states)
        # print("Status change vector: ")
        next_leg_positions = np.zeros((4,2))
        
        for i in range(4):
            if status_change[i] != 0:
                self.traj_obj[i].new_trajectory_compute(leg_velocities[i],leg_states[i],alphas[i])
            next_leg_positions[i,:] = self.traj_obj[i].next_leg_point(leg_timings[i,leg_states[i]])
        
        self.previous_leg_states = leg_states

        return next_leg_positions

class Stance_Phase_State:

    def __init__(self):

        return

class Swing_Phase_State:
    
    def __init__(self):

        return

class Inverse_Leg_Kinematics:
    # Class that handles the IK aspects of the legs

    def __init__(self):
        print("Initialized inverse leg kinematics")
        self.mouse_parameters = Mouse_Parameters()
        self.setup_leg_models()

    def setup_leg_models(self):
        front_leg_t3_param = self.mouse_parameters.fr_t1_param
        rear_leg_t3_param = self.mouse_parameters.rr_t3_param

        self.lu_fl = Leg_Unit('fr3',front_leg_t3_param)
        self.lu_fr = Leg_Unit('fr3',front_leg_t3_param)
        self.lu_rl = Leg_Unit('rr3',rear_leg_t3_param)
        self.lu_rr = Leg_Unit('rr3',rear_leg_t3_param)

    def run_inverse_leg_kinematics(self,new_target_leg_positions):
        print("Compute the necessary q_values for our leg")
        return


import numpy as np

# *******************************************************
# Type: Data Directory
# 
# Contains gait & mouse specific standard parameters
#
# Author: Alex Rohregger
# Contact: alex.rohregger@tum.de
# Last-edited: 05.06.2021
# *********************************************************


class Gait_Parameters:

    def __init__(self):
        self.set_trot_parameters()


    def set_trot_parameters(self):
        # High level parameters for the trot gait
        # Cycle_freq       [Hz]        float           - time for a full cycle
        # cycle_distr       [/]         float           - stance % vs. swing % of cycle
        # leg_cycle_offset  [/]         float⁽⁴⁾        - gait offset (start of stance within cycle)
        # Legs: 0, 1, 2, 3| FL, FR, RL, RR
        self.st_trot_parameters = {'cycle_freq': 0.67,
                                'cycle_distr': 0.5,
                                'leg_cycle_offset': np.array([0.5,0,0,0.5])}
        
        self.walk_lat_parameters = {'cycle_freq': 0.67,
                                'cycle_distr': 0.67,
                                'leg_cycle_offset': np.array([0.62,0.1,0,0.52])}
        
        self.gallop_rot_parameters = {'cycle_freq': 1.0,
                                'cycle_distr': 0.3,
                                'leg_cycle_offset': np.array([0.65,0.55,0,0.1])}

        self.walk_trot_parameters = {'cycle_freq': 1.0,
                                'cycle_distr': 0.6,
                                'leg_cycle_offset': np.array([0.5,0,0,0.5])}
        
        # Due to unsymmetric front/rear legs different in neutral heights of stance
        self.st_trot_param_f = {'cycle_freq':           self.st_trot_parameters['cycle_freq'],
                                'amp_swing':            0.01,
                                'amp_stance':           0.005,
                                'max_stride_length':    0.03,
                                'neutral_stance_pos':   -0.038,
                                'neutral_stride_pos':   -0.005,
                                'cycle_distr':          self.st_trot_parameters['cycle_distr'],
                                'leg_cycle_offset':     self.st_trot_parameters['leg_cycle_offset']}

        self.st_trot_param_r = {'cycle_freq':           self.st_trot_parameters['cycle_freq'],
                                'amp_swing':            0.015,
                                'amp_stance':           0.005,
                                'max_stride_length':    0.03,
                                'neutral_stance_pos':   -0.042,
                                'neutral_stride_pos':   -0.01,
                                'cycle_distr':          self.st_trot_parameters['cycle_distr'],
                                'leg_cycle_offset':     self.st_trot_parameters['leg_cycle_offset']}

class Mouse_Parameters:

    def __init__(self):
        self.set_mouse_parameters()

    def set_mouse_parameters(self):
        # Setting leg parameters - units [m]
        self.fr_t1_param = {'lr0':0.033, 'rp': 0.008, 'd1': 0.0128, 'l1': 0.0295, 
                            'l2': 0.0145, 'l3': 0.0225, 'l4': 0.0145,'theta3':23*np.pi/180}
        
        self.rr_t3_param = {'lr0':0.03, 'rp': 0.008, 'd1': 0.0128, 'l1': 0.0317, 
                            'l2': 0.02, 'l3': 0.0305, 'l4': 0.0205,'theta3':73*np.pi/180}

        # Setting general mouse parameters
        # [x,y,z] - units [m]
        # ^z -> y (RHS)
        self.mouse_param = {'fl_leg_attach':    np.array([0.0368,-0.02984,-0.00532]),   # From main body COM
                            'fr_leg_attach':    np.array([-0.0368,-0.02984,-0.00532]),  # From main body COM
                            'spine_attach':     np.array([0,0.03166,0.02705])           # From main body COM 
                            }

        # Mass parameters for reference - unit [kg]
        self.mouse_mass_param = {'front_mouse':     0.1538,
                                'rear_mouse':       0.0435,
                                'spine':            0.0040,
                                'head':             0.0231,
                                'legs':             0.0020,}

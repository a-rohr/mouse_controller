import rospy
import roslib
import numpy as np
import time
import tf

import sys
import threading

from std_msgs.msg import Empty, _String
from sensor_msgs.msg import Joy 
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import InteractiveMarkerInit

import os

val_jlx = rospy.get_param("/vel_lx")
val_jly = rospy.get_param("/vel_ly")


val_jrx = rospy.get_param("/vel_rx")
val_jry = rospy.get_param("/vel_ry")

val_dx = rospy.get_param("/vel_dx")
val_dy = rospy.get_param("/vel_dy")



def callback_vel(data):
    # callback to velocity
    # print(data.axes[1])
    # print(data.axes[3])
    rospy.set_param("/vel_lx",data.axes[0])
    rospy.set_param("/vel_ly",data.axes[1])
    rospy.set_param("/vel_rx",data.axes[3])
    rospy.set_param("/vel_ry",data.axes[4])
    rospy.set_param("/vel_dx",data.axes[6])
    rospy.set_param("/vel_dy",data.axes[7])
    rospy.set_param("/x_trigger",data.buttons[0])
    rospy.set_param("/o_trigger",data.buttons[1])
    rospy.set_param("/t_trigger",data.buttons[2])
    rospy.set_param("/s_trigger",data.buttons[3])



def mouse_node(rate):
    # main starter method
    # print("Starting mouse node")
    dry = 0

    # Initialize the node
    rospy.init_node("mouse_node", anonymous=True)
    r = rospy.Rate(rate)

    # Subscribe to the ps4 controller
    rospy.Subscriber("joy", Joy, callback_vel, queue_size=1)

    while(not rospy.is_shutdown()):

        # print("working")
        r.sleep()


if __name__ == "__main__":
    try:
        mouse_node(rate=60)
    except rospy.ROSInterruptException:
        pass

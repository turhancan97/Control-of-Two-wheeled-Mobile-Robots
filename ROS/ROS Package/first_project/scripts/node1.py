#!/usr/bin/env python3
# license removed for brevity
import rospy
from first_project.msg import VelocityValues
from first_project.msg import PositionControl
from math import sin, cos, pi
import math
import numpy as np


def talker():
    rospy.init_node('trajectory_generator', anonymous=True)
    pub = rospy.Publisher('velocity_values', VelocityValues,queue_size=10)
    rate = rospy.Rate(1) # 1hz
    msg = VelocityValues()
    vx_1 = float(input("Please Enter Vx:"))
    vth_1 = float(input("Please Enter Vth:"))
    trajectory_1 = input("Enter circle or stadium:")
    current_time = rospy.Time.now()
    
    while not rospy.is_shutdown():
        last_time = rospy.Time.now()
        dt = (current_time - last_time).to_sec()
        msg.vth = vth_1
        msg.vx = vx_1
        msg.trajectory = trajectory_1
        #vth = float(input("Vth:"))
        rospy.loginfo(msg)
        #rospy.loginfo(vth)
        pub.publish(msg)
        #pub.publish(vth)
        rate.sleep()
talker()
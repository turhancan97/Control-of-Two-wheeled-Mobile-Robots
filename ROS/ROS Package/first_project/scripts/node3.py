#!/usr/bin/env python3
import math
from math import sin, cos, pi
import matplotlib.pyplot as plt
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

x_temp_v = []
y_temp_v = []
x_temp_1 = []
y_temp_1 = []
x_temp_2 = []
y_temp_2 = []
while not rospy.is_shutdown():
    def odometryCb_v(msg):
        print("x_v:",msg.pose.pose.position.x)
        print("y_v:",msg.pose.pose.position.y)
        print("-----------------------------------")
        x_temp_v.append(float(msg.pose.pose.position.x))
        y_temp_v.append(float(msg.pose.pose.position.y))
    def odometryCb_1(msg):
        print("x_1:",msg.pose.pose.position.x)
        print("y_1:",msg.pose.pose.position.y)
        print("-----------------------------------")
        x_temp_1.append(float(msg.pose.pose.position.x))
        y_temp_1.append(float(msg.pose.pose.position.y))
    def odometryCb_2(msg):
        print("x_2:",msg.pose.pose.position.x)
        print("y_2:",msg.pose.pose.position.y)
        print("-----------------------------------")
        x_temp_2.append(float(msg.pose.pose.position.x))
        y_temp_2.append(float(msg.pose.pose.position.y))

    if __name__ == "__main__":
        rospy.init_node('odometry', anonymous=True) #make node 
        rospy.Subscriber('odom_v',Odometry,odometryCb_v)
        rospy.Subscriber('odom_1',Odometry,odometryCb_1)
        rospy.Subscriber('odom_2',Odometry,odometryCb_2)
        rospy.spin()

plt.plot(x_temp_v,y_temp_v,"b",label='Virtual Leader')
plt.plot(x_temp_1,y_temp_1,"r",label='First Follower')
plt.plot(x_temp_2,y_temp_2,"g",label='Second Follower')
plt.scatter(10.5,6.7,60,"black","x",label='Static Obstacle 1')
plt.scatter(9,7.15,60,"black","x",label='Static Obstacle 2')
plt.title('Trajectory Simulation',fontsize=15)
plt.xlabel('x [m]',fontsize=17)
plt.ylabel('y [m]',fontsize=17)
plt.grid(linestyle='--', linewidth=1) 
plt.legend()      
plt.show()

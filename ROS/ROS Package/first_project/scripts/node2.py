#!/usr/bin/env python3
# license removed for brevity

#import math
from math import sin, cos, pi
import matplotlib.pyplot as plt
import numpy as np
from functions import dV, dVxy, error_obs, model, k_gain, R_ , r_
import rospy
import tf
from first_project.msg import VelocityValues
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

def velo_func(velo):
    odom_pub_v = rospy.Publisher("odom_v", Odometry, queue_size=50)
    odom_pub_1 = rospy.Publisher("odom_1", Odometry, queue_size=50)
    odom_pub_2 = rospy.Publisher("odom_2", Odometry, queue_size=50)
    odom_broadcaster_v = tf.TransformBroadcaster()
    odom_broadcaster_1 = tf.TransformBroadcaster()
    odom_broadcaster_2 = tf.TransformBroadcaster()
    # %% simulation parameter
    total_time = 70 # (seconds)
    dt = 0.1; # step sizes
    time_stamp = total_time/dt # simulation time
    time = np.zeros([1,int(time_stamp)])
    i = 0
    
    # %% mobile robot parameter and initial contitions
    v_i = [0, 0, 0] # initial linear velocity of a followers (ignore first index)
    w_i = [0, 0, 0] # initial angular velocity of a followers (ignore first index)
    vy = 0 # ignore it just for ROS
    

    # %% Virtual Leader Parameters
    x_pos_v = np.zeros([3, int(time_stamp)]) # store the position
    point_v = [5,5,0] # starting point and orientation

    # %% Robot 1 (follower) parameters
    x_pos_1 = np.zeros([3, int(time_stamp)]) # store the position
    point_1 = [8,8,(3*np.pi/2)];  # starting point and orientation

    # %% Robot 2 (follower) parameters 
    x_pos_2 = np.zeros([3, int(time_stamp)]) # store the position
    point_2 = [10,10,(3*np.pi/2)] # starting point and orientation

    # %% To store the error values
    er = np.zeros([3, 3])
    er_f1 = np.zeros([3,int(time_stamp)])
    er_f2 = np.zeros([3,int(time_stamp)])
    # %% Target positions for the followers
    target_r_1 = + 1
    target_r_2 = target_r_1 + 1

    # %% k parameters for control
    k1,k2,k3 = k_gain(0.3,0.3,0.8)
    # %% Trajectory Parameter
    trajectory = velo.trajectory # % "stadium" % "circle"

    #current_time = rospy.Time.now()
    #last_time = rospy.Time.now()
    r = rospy.Rate(10.0)

    #v_r = velo.vx
    #w_r = velo.vth

    while not rospy.is_shutdown():
        # compute odometry in a typical way given the velocities of the robot
        #dt = (current_time - last_time).to_sec()
        if trajectory == "stadium":
            if i % 120 < 40: # % first line
                v_r = velo.vx # % virtual linear velocity
                w_r = 0 # % virtual angular velocity
            elif i % 120 < 60: # % first arc 
                v_r = velo.vx # % virtual linear velocity
                w_r = np.pi/2 # % virtual angular velocity
            elif i % 120 < 100: # % second line
                v_r = velo.vx # % virtual linear velocity
                w_r = 0 # % virtual angular velocity
            elif i % 120 <= 119: # % second arc
                v_r = velo.vx # % virtual linear velocity
                w_r = np.pi/2 # % virtual angular velocity
        elif trajectory == "circle":
            v_r = velo.vx # % virtual linear velocity
            w_r = velo.vth # % virtual angular velocity
        
        current_time = rospy.Time.now()
        # since all odometry is 6DOF we'll need a quaternion created from yaw
        odom_quat_v = tf.transformations.quaternion_from_euler(0, 0, point_v[2])

        # first, we'll publish the transform over tf
        odom_broadcaster_v.sendTransform(
            (point_v[0], point_v[1], 0.),
            odom_quat_v,
            current_time,
            "base_link",
            "odom_v"
        )

        # next, we'll publish the odometry message over ROS
        odom_v = Odometry()
        odom_v.header.stamp = current_time
        odom_v.header.frame_id = "odom"

        # set the position
        odom_v.pose.pose = Pose(Point(point_v[0], point_v[1], 0.), Quaternion(*odom_quat_v))

        # set the velocity
        odom_v.child_frame_id = "base_link"
        odom_v.twist.twist = Twist(Vector3(v_r, vy, 0), Vector3(0, 0, w_r))

        # publish the message
        odom_pub_v.publish(odom_v)

        #last_time = current_time
        r.sleep()
        # % Virtual Leader Process (1.robot işlemleri yapılıyor)

        
        x_pos_v[:,i] = point_v
        q_dot = model(point_v[2],v_r,w_r) # % time derivative of position
        point_v[0:2] = np.add(point_v[0:2],([q_dot[0][0]*dt,q_dot[1][0]*dt])) # % state (position) update
        point_v[2] = np.add(point_v[2],(q_dot[2][0] * dt)) # % state (orientation) update
        # % 1st and 2nd follower process (2. ve 3. robot işlemleri)
        for j in range(1,3,1): # % simulate the followers here 
            if j == 1: # % for 1st follower
                current_time = rospy.Time.now()
                odom_quat_1 = tf.transformations.quaternion_from_euler(0, 0, point_1[2])
                # first, we'll publish the transform over tf
                odom_broadcaster_1.sendTransform(
                    (point_1[0], point_1[1], 0.),
                    odom_quat_1,
                    current_time,
                    "base_link",
                    "odom_1"
                )

                # next, we'll publish the odometry message over ROS
                odom_1 = Odometry()
                odom_1.header.stamp = current_time
                odom_1.header.frame_id = "odom"

                # set the position
                odom_1.pose.pose = Pose(Point(point_1[0], point_1[1], 0.), Quaternion(*odom_quat_1))

                # set the velocity
                odom_1.child_frame_id = "base_link"
                odom_1.twist.twist = Twist(Vector3(v_i[j], vy, 0), Vector3(0, 0, w_i[j]))

                # publish the message
                odom_pub_1.publish(odom_1)

                #last_time = current_time
                r.sleep()
                x_pos_1[:,i] = point_1
                q_dot = model(point_1[2],v_i[j],w_i[j]) # % time derivative of position
                point_1[0:2] = np.add(point_1[0:2],([q_dot[0][0]*dt,q_dot[1][0]*dt])) # % state (position) update
                point_1[2] = np.add(point_1[2],(q_dot[2][0] * dt)) # % state (orientation) update
                # % Obstacle Avoidance
                # % x_pos(coordinate, time step , robot number)
                xr = x_pos_1[:,i][0] # % x coordinate of 1st follower 
                yr = x_pos_1[:,i][1] # % y coordinate of 1st follower
                xobs1 = x_pos_v[:,i][0] # % x coordinate of virtual leader (Obstacle for followers)
                yobs1 = x_pos_v[:,i][1] # % y coordinate of virtual leader (Obstacle for followers)
                dV1 = dVxy([xr,yr,xobs1,yobs1]) # % APF function for virtual leader
                dV2 = dVxy([xr,yr,10.5,6.7]) # % APF function for static obstacle (x = 10.5, y = 6.7)
                dV3 = dVxy([xr,yr,9,7.15]) # % APF function for static obstacle (x = 9, y = 7.15)
                dV4 = dV1 + dV2 + dV3 # % sum of all obstacle
                # % Control Algorithm
                # % error_obs(x of virtual learder, y of virtual leader - 1, virtual leader theta, x of follower 1, y of follower 1, theta of follower 1, APF x, APF y)
                e = error_obs(x_pos_v[:,i][0],x_pos_v[:,i][1]-target_r_1,x_pos_v[:,i][2],x_pos_1[:,i][0],x_pos_1[:,i][1],x_pos_1[:,i][2],dV4[0],dV4[1])
                er[:,j] = e.reshape(3,) # % for control algorithm
                er_f1[:,i] = e.reshape(3,)
                # err(:,i,1) = e # % for plotting 
                v_i[j] = v_r * np.cos(er[2,j]) + k1 * (er[0,j]) # % Control Algorithm from Canudas et al. (1994)
                w_i[j] = w_r + (k2 * (er[1,j]) + (k3 *er[2,j])) # % Control Algorithm from Canudas et al. (1994)
            else:  # % for second follower
                current_time = rospy.Time.now()
                odom_quat_2 = tf.transformations.quaternion_from_euler(0, 0, point_2[2])
                # first, we'll publish the transform over tf
                odom_broadcaster_2.sendTransform(
                    (point_2[0], point_2[1], 0.),
                    odom_quat_2,
                    current_time,
                    "base_link",
                    "odom_2"
                )

                # next, we'll publish the odometry message over ROS
                odom_2 = Odometry()
                odom_2.header.stamp = current_time
                odom_2.header.frame_id = "odom"

                # set the position
                odom_2.pose.pose = Pose(Point(point_2[0], point_2[1], 0.), Quaternion(*odom_quat_2))

                # set the velocity
                odom_2.child_frame_id = "base_link"
                odom_2.twist.twist = Twist(Vector3(v_i[j], vy, 0), Vector3(0, 0, w_i[j]))

                # publish the message
                odom_pub_2.publish(odom_2)

                #last_time = current_time
                r.sleep()
                x_pos_2[:,i] = point_2
                q_dot = model(point_2[2],v_i[j],w_i[j]) # % time derivative of position
                point_2[0:2] = np.add(point_2[0:2],([q_dot[0][0]*dt,q_dot[1][0]*dt])) # % state (position) update
                point_2[2] = np.add(point_2[2],(q_dot[2][0] * dt)) # % state (orientation) update
                # % Obstacle Avoidance
                # % x_pos(coordinate, time step , robot number)
                xr = x_pos_2[:,i][0] # % x coordinate of 2nd follower 
                yr = x_pos_2[:,i][1] # % y coordinate of 2nd follower
                xobs1 = x_pos_v[:,i][0] # % x coordinate of virtual leader (Obstacle for followers)
                yobs1 = x_pos_v[:,i][1] # % y coordinate of virtual leader (Obstacle for followers)
                xobs2 = x_pos_1[:,i][0] # % x coordinate of 1st follower (Obstacle for followers)
                yobs2 = x_pos_1[:,i][1] # % y coordinate of 1st follower (Obstacle for followers)
                dV1 = dVxy([xr,yr,xobs1,yobs1]) # % APF function for virtual leader
                dV2 = dVxy([xr,yr,xobs2,yobs2]) # % APF function for 1st follower
                dV3 = dVxy([xr,yr,10.5,6.7]) # % APF function for static obstacle (x = 10.5, y = 6.7)
                dV4 = dVxy([xr,yr,9,7.15]) # % APF function for static obstacle (x = 9, y = 7.15)
                dV5 = dV1 + dV2 + dV3 + dV4 # % sum of all obstacle
                # % Control Algorithm
                # % error_obs(x of virtual learder, y of virtual leader - 2, virtual leader theta, x of follower 2, y of follower 2, theta of follower 2, APF x, APF y)
                e = error_obs(x_pos_v[:,i][0],x_pos_v[:,i][1]-target_r_2,x_pos_v[:,i][2],x_pos_2[:,i][0],x_pos_2[:,i][1],x_pos_2[:,i][2],dV5[0],dV5[1])
                er[:,j] = e.reshape(3,) # % for control algorithm
                er_f2[:,i] = e.reshape(3,)
                # err(:,i,1) = e # % for plotting 
                v_i[j] = v_r * np.cos(er[2,j]) + k1 * (er[0,j]) # % Control Algorithm from Canudas et al. (1994)
                w_i[j] = w_r + (k2 * (er[1,j]) + (k3 *er[2,j])) # % Control Algorithm from Canudas et al. (1994)
        time[:,i] = i/10
        i = i + 1

def listener():
    rospy.init_node('odometry_publisher')
    rospy.Subscriber('velocity_values',VelocityValues,velo_func)
    rospy.spin()
listener()
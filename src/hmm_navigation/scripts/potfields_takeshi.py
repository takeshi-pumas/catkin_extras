#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Apr 17 22:54:18 2019
Updated on Mon May 06 15:02:03 2024

@author: oscar
"""

import rospy
import numpy as np
from geometry_msgs.msg import Twist, PointStamped
from sensor_msgs.msg import LaserScan
import tf2_ros
from tf.transformations import euler_from_quaternion
# from std_msgs.msg import String
# from nav_msgs.msg import Odometry
# import tf as tf

xcl,ycl = 0,0
Fx_rep, Fy_rep = 0,0
cont = 0
speed = Twist()
speed.angular.z = 0

# Transform tf message to np arrays
def tf_2_np_array(t):
    pose = np.asarray((
        t.transform.translation.x,
        t.transform.translation.y,
        t.transform.translation.z))
    quat = np.asarray((
        t.transform.rotation.x,
        t.transform.rotation.y,
        t.transform.rotation.z,
        t.transform.rotation.w))
    return pose, quat

def calculate_force(current_position, goal):
    pass

# Suscribers callbacks

def read_point_cb(msg):
    global xcl,ycl
    xcl = msg.point.x
    ycl = msg.point.y
           
def read_sensor_cb(msg):
    # This callback is about to be shorten
    global cont, xcl,ycl, Fx_rep, Fy_rep
    lectures = np.asarray(msg.ranges)
    lectures[np.isinf(lectures)] = 13.5

    # No entiendo por que Fy_rep no inicia en 0 como los otros !!!
    Fx_rep = 0.0
    Fy_rep = 0.001
    Fth_rep = 0.0

    # Calculate repulsive force
    for idx, deg in enumerate(laser_degs):            
        Fx_rep = Fx_rep + (1/lectures[idx])**2 * np.cos(deg)
        Fy_rep = Fy_rep + (1/lectures[idx])**2 * np.sin(deg)
    
    # No entiendo por que hay que sumar una cantidad muy pequeña !!!
    Fth_rep = np.arctan2(Fy_rep, (Fx_rep + 0.000000000001)) + np.pi
    Fmag_rep = np.linalg.norm((Fx_rep, Fy_rep))

    # Get current robot position
    try:
        trans = tfBuffer.lookup_transform('map','base_footprint', rospy.Time(), rospy.Duration(5.0))
        pose, quat = tf_2_np_array(trans)
        x, y = pose[0], pose[1]
        euler = euler_from_quaternion(quat)
        th = euler[2]
    
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.WARN('Waiting for TF')
        x, y = 0,0
        th = 0


    # Calculate atractive force
    xy, xycl = np.array((x,y)), np.array((xcl,ycl))
    euclD = np.linalg.norm(xy - xycl)

    Fx_atr = -(x - xcl) / euclD
    Fy_atr = -(y - ycl) / euclD      
    Fth_atr = np.arctan2(Fy_atr, Fx_atr) 
    Fth_atr = Fth_atr - th
    Fmag_atr = np.linalg.norm((Fx_atr, Fy_atr))
    Fx_tot = Fmag_rep * np.cos(Fth_rep) * 0.0025 + Fmag_atr * np.cos(Fth_atr)
    Fy_tot = Fmag_rep * np.sin(Fth_rep) * 0.0025 + Fmag_atr * np.sin(Fth_atr)
    Fth_tot = np.arctan2(Fy_tot, Fx_tot)
     
    
    ## Esos 2 if se pueden cambiar ....
    '''if ( Fth_tot > np.pi ):
        Fth_tot = -np.pi - (Fth_tot-np.pi) ## <-- creo que la operacion correcta es Fth_tot - 2*np.pi

    if (Fth_tot < -np.pi):
        Fth_tot = (Fth_tot + 2 * np.pi)'''

    # .... Por solo un if, se necesita probar
    # Tal vez no se necesita porque np.arctan2 regresa valores acotados entre -pi y pi 
    '''if abs(Fth_tot) > np.pi:
        Fth_tot -= 2 * np.pi * np.sign(Fth_tot)'''

    if (xcl!=0 and ycl!=0):
        print("xrob, yrob, throbot: {:.2f}, {:.2f}, {:.2f}".format(x, y, np.rad2deg(th)))
        print("xclick, yclick: {:.2f}, {:.2f}, euclD: {:.2f}".format(xcl, ycl, euclD))
        print("Repulsive Force: Fx, Fy, Fth: {:.2f}, {:.2f}, {:.2f}".format(Fx_rep, Fy_rep, np.rad2deg(Fth_rep)))
        print("Atractive Force: Fx, Fy, Fth: {:.2f}, {:.2f}, {:.2f}".format(Fx_atr, Fy_atr,  np.rad2deg(Fth_atr)))
        print("Total force Fx, Fy, Tth: {:.2f}, {:.2f}".format(Fx_tot, Fy_tot, np.rad2deg(Fth_tot)))

        # Stop condition
        proximity_threshold = 0.15
        if(euclD < proximity_threshold):
            speed.linear.x = 0
            speed.linear.y = 0
            speed.angular.z = 0
            xcl, ycl = 0.0, 0.0

        else:
            # Speed behaviors 
            if( abs(Fth_tot) < 0.27) :#or (np.linalg.norm((Fx,Fy)) < 100):
                speed.linear.x =  min(current_speed.linear.x + 0.0015, 0.5)
                speed.angular.z = 0
                print('lin')
            else:
                if Fth_tot > -np.pi/2  and Fth_tot < 0:
                    print('Vang-')
                    speed.linear.x  = max(current_speed.linear.x - 0.0003, 0.05)
                    speed.angular.z = max(current_speed.angular.z - 0.0005, -0.2)
                
                elif Fth_tot < np.pi/2  and Fth_tot > 0:
                    print('Vang+')
                    speed.linear.x  = max(current_speed.linear.x - 0.0003, 0.05)
                    speed.angular.z = min(current_speed.angular.z + 0.0005, 0.2)
                
                elif Fth_tot < -np.pi/2:
                    print('Vang---')
                    speed.linear.x  = max(current_speed.linear.x - 0.0035, 0.002)
                    speed.angular.z = max(current_speed.angular.z - 0.003, -0.5)

                elif Fth_tot > np.pi/2:
                    print('Vang+++')
                    speed.linear.x  = max(current_speed.linear.x - 0.0035, 0.002)
                    speed.angular.z = min(current_speed.angular.z + 0.003, 0.5)
    else:      
        cont += 1
        if cont == 200:
            print('Waiting for goal clicked point')
            cont = 0


# Cambio de nombre de inoutinout a main
def main():
    global listener, tfBuffer, xcl, ycl, current_speed, cont, laser_degs

    rospy.init_node('pot_fields_nav', anonymous = True)

    # Setup variables
    rate = rospy.Rate(25) # 10hz
    data = rospy.wait_for_message('/hsrb/base_scan', LaserScan, timeout=5)
    laser_degs = np.linspace(data.angle_min, data.angle_max, len(data.ranges))

    # Subscribers and publishers
    laser_base_sub = rospy.Subscriber("/hsrb/base_scan", LaserScan, read_sensor_cb)
    clicked_point_sub = rospy.Subscriber("/clicked_point", PointStamped, read_point_cb)
    cmd_vel_pub = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=1)
    
    # TF buffer and listener
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    
    rospy.loginfo('Pot Fields Navigation AMCL active')

    while not rospy.is_shutdown():
        if (xcl != 0 and ycl != 0):
            
            cmd_vel_pub.publish(speed)
            current_speed = speed
            rate.sleep()
        else:
            current_speed = Twist()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on june 24
@author: oscar
"""

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist ,Point32
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan,PointCloud, PointCloud2
import tf2_ros
from tf2_geometry_msgs import PointStamped
import tf2_geometry_msgs

import ros_numpy
import numpy as np
import tf as tf
import struct
xcl,ycl=0,0
cont=0






def write_Point_Stamped(xy=[0,0]):
    """
    Creates a PointStamped object with the given x and y coordinates.

    Args:
        xy (list, optional): The x and y coordinates. Defaults to [0, 0].

    Returns:
        PointStamped: The created PointStamped object.
    """
    point_st = PointStamped()
    point_st.header.frame_id = "odom"#"base_range_sensor_link"
    point_st.point.x = xy[0]
    point_st.point.y = xy[1]  # replace with your y coordinate
    point_st.point.z =  0.0 ###FLOOR ### humanpose.z  # replace with your z coordinate
    return point_st



def get_avoid_chairs_force(msg,K_tune=10):
    """
    Calculates the repulsive force to avoid chairs based on laser scan data.

    Args:
        msg (LaserScan): The laser scan data.
        K_tune (int, optional): Tuning parameter for the repulsive force. Defaults to 10.

    Returns:
        numpy.ndarray: The repulsive force vector.
    """
    #cloud_pub = rospy.Publisher('cloud_topic', PointCloud, queue_size=10)
    global cloud_pub
    #msg=rospy.wait_for_message('/hsrb/base_scan', LaserScan)
    cloud = PointCloud()
    cloud.header.frame_id = "base_range_sensor_link" # "map"
    F_rep=[]
    repulsors=[]
    angles= np.flip(np.arange(msg.angle_min, msg.angle_max,msg.angle_increment ))
    ranges=np.asarray(msg.ranges)
    spikes=np.diff(np.nan_to_num(ranges))
    spikes[spikes> 10]=10
    spikes[spikes< -10]=-10
    non_zero_spikes=np.where(abs(spikes)>=0.5)
    for spike in non_zero_spikes[0]:repulsors.append(spike)    
    for spike_n in repulsors:
        range_reg=ranges[spike_n-2:spike_n+1]
        mask = np.isfinite(range_reg)
        if mask.any(): 
            pt_xy= [np.cos(angles[spike_n])*np.nanmean(range_reg[mask]),np.sin(angles[spike_n])*np.nanmean(range_reg[mask])]
            frep=np.asarray((pt_xy))
            
            d_torep=np.linalg.norm(frep)
            if d_torep <1.0:  #TUNABLE
                F_rep.append(frep/d_torep**3)
                pt = write_Point_Stamped(pt_xy)
                
                point_odom = tfBuffer.transform(pt, "base_range_sensor_link", timeout=rospy.Duration(1))
                #point_odom = tfBuffer.transform(pt, "map", timeout=rospy.Duration(1))
                cloud.points.append(Point32(point_odom.point.x, -point_odom.point.y, point_odom.point.z))

    
    if len (F_rep)!=0:F_rep=np.asarray(F_rep).sum(axis=0)
    else:
        F_rep=np.zeros(2)
        #cloud.points.append(Point32(0,0, -0.5))
    cloud_pub.publish(cloud)
    F_rep[1]=F_rep[1]*-1
    return F_rep* -K_tune



def get_rep_force(msg):
    """
    Calculates the repulsive force based on the laser scan data.

    Args:
        msg: The LaserScan message containing the range and angle data.

    Returns:
        numpy.ndarray: An array representing the repulsive force in x and y directions.
    """
    
    lec = np.asarray(msg.ranges)
    laserdegs = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
    Fx = 0
    Fy = 0.0
    for i, deg in enumerate(laserdegs):
        if lec[i] < 1.5:
            Fx = Fx + (1 / lec[i]) ** 2 * np.cos(deg) * -1    ###
            Fy = Fy + (1 / lec[i]) ** 2 * np.sin(deg) * -1    ### Direction of Force opposite to ray

    Fmag = np.linalg.norm((Fx, Fy))
    Ffas = np.arctan2(Fx, Fy)
    return np.asarray((Fx, Fy))


def get_attr_force():
    #F_atr= np.asarray ((xcl-x,ycl-y)) 
    print (x,y,xcl,ycl,'atrracted to')
    Fatrx =( -x + xcl)
    Fatry =( -y + ycl)
    Fatrmag=np.linalg.norm((Fatrx,Fatry))
    Fatrth=np.arctan2(Fatry, Fatrx) 
    Fatrth=Fatrth-th
    
    return np.asarray((Fatrmag*np.cos(Fatrth), Fatrmag*np.sin(Fatrth)))

def newOdom (msg):
    global x
    global y
    global th
    
    x=msg.pose.pose.position.x
    y=msg.pose.pose.position.y
    quaternion = (
    msg.pose.pose.orientation.x,
    msg.pose.pose.orientation.y,
    msg.pose.pose.orientation.z,
    msg.pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    th = euler[2]

def readPoint(punto):
    global xcl, ycl, pt_cl
    xcl = punto.point.x
    ycl = punto.point.y
    pt_cl = punto
    print(f'Goal clicked point: {xcl, ycl}')

def readSensor(msg):
    global cont, xcl,ycl , current_speed
    #
    points_data = ros_numpy.numpify(msg)
    ###
    # Extract XYZ coordinates
    x = np.nansum(points_data['x'])
    y = np.nansum(points_data['y'])
    z = np.nansum(points_data['z'])
    Fxyz=np.asarray((x,y,z))
    F_rep_xyz=Fxyz/np.linalg.norm(Fxyz)
    print ('F HEAd',F_rep_xyz)



     # Define a point in head_rgbd_sensor_rgb_frame
    point_in_head = PointStamped()
    point_in_head.header.frame_id = "head_rgbd_sensor_rgb_frame"
    point_in_head.point.x = F_rep_xyz[0]
    point_in_head.point.y = F_rep_xyz[1]
    point_in_head.point.z = F_rep_xyz[2]

     
    try:
        inverted_transform = tfBuffer.lookup_transform("base_link", "head_rgbd_sensor_rgb_frame", rospy.Time(0), rospy.Duration(1.0))

    except:
        print('No tf, error') 
        inverted_transform = np.asarray((0,0,0,1))
    ###

    point_in_base = tf2_geometry_msgs.do_transform_point(point_in_head, inverted_transform)
    print (point_in_base,'F REP BASE')

    if 'x' not in globals():
        print ('waiting for odom')
    else:
        print ('odom received',xcl,ycl)
        euclD= np.linalg.norm((-xcl+x,-ycl+y))
        F_atr=get_attr_force()

        #F_rep=get_rep_force(data)
        #F_rep_chairs= get_avoid_chairs_force(data)
        F_rep, F_rep_chairs=0,0 ## REMOVE
        
        print ( F_atr,F_rep,F_rep_chairs,"F_atr,F_rep,F_rep_chairs")
        Ftotx,Ftoty=  10*F_atr+ 0.025*F_rep + F_rep_chairs
        Ftotth=np.arctan2(Ftoty,Ftotx)
        Ftotth = np.fmod(Ftotth + np.pi, 2 * np.pi) - np.pi        
        print('Ftotxy',Ftotx,Ftoty,Ftotth)
        euclD= np.linalg.norm((-xcl+x,-ycl+y))
        print  ("euclD",euclD)
        print("Ftotth",Ftotth)
        if (xcl!=0 and ycl!=0):
            vel=0.07
            if (euclD < 0.5) :####################HOW CLOSE?
                speed.linear.x=0
                speed.linear.y=0
                speed.angular.z=0
                xcl,ycl=0.0 , 0.0
            else:
                if( abs(Ftotth) < .35) :
                    speed.linear.x=  min (current_speed.linear.x+0.0015, 2.0)  #0.5)#1.9   MAX SPEED
                    if abs (F_atr[1])<0.1:
                        speed.angular.z= np.clip(0.5 * F_atr[1], -0.5, 0.5)
                        print(f'correcting heading{speed.angular.z}')
                    print('lin')
                else:
                    if Ftotth > -np.pi/2  and Ftotth <0:
                        print('Vang-')
                        speed.linear.x  = max(current_speed.linear.x -0.0003, 0.08)
                        speed.angular.z = max(current_speed.angular.z-0.003, -0.15)

                    if Ftotth < np.pi/2  and Ftotth > 0:
                        print('Vang+')
                        speed.linear.x  = max(current_speed.linear.x-0.0003, 0.04)
                        speed.angular.z = min(current_speed.angular.z+0.003,0.15)


                    if Ftotth < -np.pi/2:

                        print('Vang---')
                        speed.linear.x  = max(current_speed.linear.x-0.025, 0.0)
                        speed.angular.z = max(current_speed.angular.z-0.06,-0.38)


                    if Ftotth > np.pi/2:

                        print('Vang+++')
                        speed.linear.x  = max(current_speed.linear.x-0.025, 0.0)
                        speed.angular.z = min(current_speed.angular.z+0.06, 0.38)
            
        print (speed.linear.x , speed.angular.z)
        #current_speed=speed
    """def readSensor(data):
                global cont, xcl,ycl , current_speed
                
                lec=np.asarray(data.ranges)
                lec[np.isinf(lec)]=13.5
                
                
                if 'x' not in globals():
                    print ('waiting for odom')
                else:
                    print ('odom received',xcl,ycl)
                    euclD= np.linalg.norm((-xcl+x,-ycl+y))
                    F_atr=get_attr_force()
                    F_rep=get_rep_force(data)
                    F_rep_chairs= get_avoid_chairs_force(data)
                    print ( F_atr,F_rep,F_rep_chairs,"F_atr,F_rep,F_rep_chairs")
                    Ftotx,Ftoty=  10*F_atr+ 0.025*F_rep + F_rep_chairs
                    Ftotth=np.arctan2(Ftoty,Ftotx)
                    Ftotth = np.fmod(Ftotth + np.pi, 2 * np.pi) - np.pi        
                    print('Ftotxy',Ftotx,Ftoty,Ftotth)
                    euclD= np.linalg.norm((-xcl+x,-ycl+y))
                    print  ("euclD",euclD)
                    print("Ftotth",Ftotth)
                    if (xcl!=0 and ycl!=0):
                        vel=0.07
                        if (euclD < 0.5) :####################HOW CLOSE?
                            speed.linear.x=0
                            speed.linear.y=0
                            speed.angular.z=0
                            xcl,ycl=0.0 , 0.0
                        else:
                            if( abs(Ftotth) < .35) :
                                speed.linear.x=  min (current_speed.linear.x+0.0015, 2.0)  #0.5)#1.9   MAX SPEED
                                if abs (F_atr[1])<0.1:
                                    speed.angular.z= np.clip(0.5 * F_atr[1], -0.5, 0.5)
                                    print(f'correcting heading{speed.angular.z}')
                                print('lin')
                            else:
                                if Ftotth > -np.pi/2  and Ftotth <0:
                                    print('Vang-')
                                    speed.linear.x  = max(current_speed.linear.x -0.0003, 0.08)
                                    speed.angular.z = max(current_speed.angular.z-0.003, -0.15)
            
                                if Ftotth < np.pi/2  and Ftotth > 0:
                                    print('Vang+')
                                    speed.linear.x  = max(current_speed.linear.x-0.0003, 0.04)
                                    speed.angular.z = min(current_speed.angular.z+0.003,0.15)
            
            
                                if Ftotth < -np.pi/2:
            
                                    print('Vang---')
                                    speed.linear.x  = max(current_speed.linear.x-0.025, 0.0)
                                    speed.angular.z = max(current_speed.angular.z-0.06,-0.38)
            
            
                                if Ftotth > np.pi/2:
            
                                    print('Vang+++')
                                    speed.linear.x  = max(current_speed.linear.x-0.025, 0.0)
                                    speed.angular.z = min(current_speed.angular.z+0.06, 0.38)
                        
                    print (speed.linear.x , speed.angular.z)
                    #current_speed=speed
                """
     

speed=Twist()
speed.angular.z=0
def inoutinout():
    
    global listener,tfBuffer,cloud_pub,xcl,ycl,current_speed,cont
    rospy.init_node('pot_fields_nomap', anonymous=True)

#############################
    pub = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=1)
    cloud_pub = rospy.Publisher('cloud_topic', PointCloud, queue_size=10)
    tfBuffer = tf2_ros.Buffer() 
    listener = tf2_ros.TransformListener(tfBuffer)    
########################################################
    rospy.Subscriber("/hsrb/wheel_odom",Odometry,newOdom)
    #rospy.Subscriber("/hsrb/odom",Odometry,newOdom)
    #rospy.Subscriber("/hsrb/base_scan",LaserScan,readSensor)
    rospy.Subscriber("/hsrb/head_rgbd_sensor/depth_registered/rectified_points",PointCloud2, readSensor)
    rospy.Subscriber("/clicked_point",PointStamped,readPoint)
    pub = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=1)
    
    rate = rospy.Rate(25) # 10hz
    print('Pot Fields AMCL active')
    while not rospy.is_shutdown():
        #print (current_speed)
        if (xcl!=0 and ycl!=0):

            
            pub.publish(speed)
            rospy.sleep(0.15)
            current_speed=speed
        else:
            print ('Waiting for goal clicked point')
            current_speed=Twist()

if __name__ == '__main__':
    try:
        inoutinout()

    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Apr 17 22:54:18 2019
@author: oscar
Edited on Tuesday October 31 2023
@editor: Daniel_V
"""
import rospy
import rospkg
from visualization_msgs.msg import Marker, MarkerArray
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA, Int16
from sensor_msgs.msg import LaserScan
import numpy as np


file_path = rospkg.RosPack().get_path('hmm_navigation') + '/scripts/hmm_nav/'
print (file_path)
centroids = np.load(file_path + 'ccvk.npy')
ccxyth = np.load(file_path + 'ccxyth.npy')
matrix_A = np.load(file_path + 'A.npy')
print (f' A{matrix_A.shape}, ccxyth{ccxyth.shape},centroids{centroids.shape}')


def get_marker3(id):
    marker3 = Marker()
    marker3.id = id
    marker3.header.frame_id = 'map'
    marker3.type = Marker.LINE_LIST
    marker3.ns = 'edges'
    marker3.action = Marker.ADD
    marker3.scale.x = 0.015
    marker3.pose.orientation.x = 0.0
    marker3.pose.orientation.y = 0.0
    marker3.pose.orientation.z = 0.0
    marker3.pose.orientation.w = 1.0
    return marker3


def get_text_marker(id, scale):
    marker = Marker()
    marker.type = Marker.TEXT_VIEW_FACING
    marker.id = id
    marker.scale.x = scale
    marker.scale.y = scale
    marker.scale.z = scale
    marker.header.frame_id = 'map'
    marker.color.a = 1.0
    marker.color.r = 0.6
    marker.color.g = 0.3
    marker.color.b = 0.3
    marker.text = "S:" + str(id)
    return marker


def callback(laser):
    global ccxyth, centroids, matrix_A ,  first,marker_array
    if first:
        first=False
        marker_array = MarkerArray()
        start_point = Point()
        end_point = Point()  # end point

        line_color = ColorRGBA()  # a nice color for my line (royalblue)
        line_color.r = 0.254902
        line_color.g = 0.411765
        line_color.b = 0.882353
        line_color.a = 1.0

        # Topological map
        id_count = 0
        for s1 in np.arange(len(matrix_A)):
            for s2 in np.arange(len(matrix_A)):
                if (s1 != s2) and (matrix_A[s1, s2] != 0):
                    print (f's1 ,{s1,ccxyth[s1,0]},s2    {s2,ccxyth[s2,0]} ')
                    start_point, end_point = Point(), Point()
                    start_point.x =  ccxyth[s1,0]
                    start_point.y = ccxyth[s1,1]
                    start_point.z = 0.1
                    end_point.x = ccxyth[s2,0]
                    end_point.y = ccxyth[s2,1]
                    end_point.z = 0.1

                    marker3 = get_marker3(id_count)
                    marker3.colors.append(line_color)
                    marker3.colors.append(line_color)
                    marker3.points.append(start_point)
                    marker3.points.append(end_point)
                    marker_array.markers.append(marker3)
                    start_point, end_point = Point(), Point()
                    id_count += 1

        # NODES
        for n in range(len(ccxyth)):
            quaternion = quaternion_from_euler(0, 0, ccxyth[n][2])
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.id = n + len(ccxyth)
            marker.type = marker.ARROW
            marker.action = marker.ADD
            marker.scale.x = 0.1
            marker.scale.y = 0.01
            marker.scale.z = 0.1
            marker.color.a = 0.8
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0
            marker.pose.orientation.x = quaternion[0]
            marker.pose.orientation.y = quaternion[1]
            marker.pose.orientation.z = quaternion[2]
            marker.pose.orientation.w = quaternion[3]
            marker.pose.position.x = ccxyth[n][0]
            marker.pose.position.y = ccxyth[n][1]
            marker.pose.position.z = 0.1
            marker.lifetime.nsecs = 1
            marker_array.markers.append(marker)

            text_marker = get_text_marker(n, 0.2)
            text_marker.pose.orientation.x = quaternion[0]
            text_marker.pose.orientation.y = quaternion[1]
            text_marker.pose.orientation.z = quaternion[2]
            text_marker.pose.orientation.w = quaternion[3]
            text_marker.pose.position.x = ccxyth[n][0]
            text_marker.pose.position.y = ccxyth[n][1]
            text_marker.pose.position.z = 0.2
            marker_array.markers.append(text_marker)

    pub_marker.publish(marker_array)

    # GET symbol
    lec = np.asarray(laser.ranges)
    lec[np.isinf(lec)] = 13.5
    symbol = np.power(lec.T - centroids[:,:721], 2).sum(axis=1, keepdims=True).argmin()
    pub_symbol.publish(symbol)


if __name__ == '__main__':
    global pub_marker, pub_symbol, first
    first= True
    rospy.init_node('show_topo_path', anonymous=True)
    rospy.loginfo("node show_topo_path started")
    rospy.Subscriber("/hsrb/base_scan", LaserScan, callback, queue_size=2)
    pub_marker = rospy.Publisher(
        '/hmm_nav/HMM_topo/', MarkerArray, queue_size=1)
    pub_symbol = rospy.Publisher(
        '/hmm_nav/Quantized_Symbol/', Int16, queue_size=1)

    rospy.spin()

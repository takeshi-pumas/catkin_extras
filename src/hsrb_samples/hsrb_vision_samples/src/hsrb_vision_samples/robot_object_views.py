#!/usr/bin/env python
# Copyright (C) 2019 Toyota Motor Corporation

import random
import math
import numpy as np
from scipy.spatial.distance import pdist
import argparse

import rospy
import sys
import os
import time
import string
import warnings
import re

import tf2_ros
import traceback

from gazebo_ros import gazebo_interface

from gazebo_msgs.msg import *
from gazebo_msgs.srv import *
from std_srvs.srv import Empty
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Wrench
import tf.transformations as tft

#hsr libraries
import hsrb_interface

from hsrb_interface import geometry
from geometry_msgs.msg import Vector3Stamped, Pose, PoseStamped, PointStamped
from geometry_msgs.msg import WrenchStamped, Twist

import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

robot = hsrb_interface.Robot()
whole_body = robot.get("whole_body")
omni_base = robot.get("omni_base") #Standard initialisation (Toyota)
gripper = robot.get('gripper')
tf_buffer = robot._get_tf2_buffer()

#erasers libraries
#from erasers_nav_msgs.srv import GetObjectsCentroid

whole_body.move_to_neutral()
rospy.loginfo('initializing...')
rospy.sleep(3)

_rgb_mat = 0
_rgb = False

_path_xml = "/home/oscar/Codes/ycb_ws/src/robot_object_views/robotobject/models/MODEL_NAME/model-1_4.sdf"
_path_model = "/home/oscar/Codes/ycb_ws/src/robot_object_views/robotobject/models"

model_database_template = """<sdf version="1.4">
  <world name="default">
    <include>
      <uri>model://MODEL_NAME</uri>
    </include>
  </world>
</sdf>"""


objects = {
    "items": [
        "002_master_chef_can",       # 000 food
        "003_cracker_box",           # 001 food
        "004_sugar_box",             # 002 food
        "005_tomato_soup_can",       # 003 food
        "006_mustard_bottle",        # 004 food
        "007_tuna_fish_can",         # 005 food
        "008_pudding_box",           # 006 food
        "009_gelatin_box",           # 007 food
        "010_potted_meat_can",       # 008 food
        "011_banana",                # 009 food
        "012_strawberry",            # 010 food
        "013_apple",                 # 011 food
        "014_lemon",                 # 012 food
        "015_peach",                 # 013 food
        "016_pear",                  # 014 food
        "017_orange",                # 015 food
        "018_plum",                  # 016 food
        "019_pitcher_base",          # 017 kitchen_items
        "021_bleach_cleanser",       # 018 kitchen_items
        "022_windex_bottle",         # 019 kitchen_items
        "024_bowl",                  # 020 kitchen_items
        "025_mug",                   # 021 kitchen_items
        "026_sponge",                # 022 kitchen_items
        "029_plate",                 # 023 kitchen_items
        "030_fork",                  # 024 kitchen_items
        "031_spoon",                 # 025 kitchen_items
        "033_spatula",               # 026 kitchen_items
        # missing: Pitcher lid, Wine glass
        "038_padlock",               # 027 tools
        "040_large_marker",          # 028 tools
        "050_medium_clamp",          # 029 tools
        "051_large_clamp",           # 030 tools
        "052_extra_large_clamp",     # 031 tools
        # missing: Small marker, Small clamp, Bolts, Nuts
        "053_mini_soccer_ball",      # 032 shape_items
        "054_softball",              # 033 shape_items
        "055_baseball",              # 034 shape_items
        "056_tennis_ball",           # 035 shape_items
        "057_racquetball",           # 036 shape_items
        "058_golf_ball",             # 037 shape_items
        "059_chain",                 # 038 shape_items
        "061_foam_brick",            # 039 shape_items
        "062_dice",                  # 040 shape_items
        "063-a_marbles",             # 041 shape_items
        "063-b_marbles",             # 042 shape_items
        "065-a_cups",                # 043 shape_items
        "065-b_cups",                # 044 shape_items
        "065-c_cups",                # 045 shape_items
        "065-d_cups",                # 046 shape_items
        "065-e_cups",                # 047 shape_items
        "065-f_cups",                # 048 shape_items
        "065-g_cups",                # 049 shape_items
        "065-h_cups",                # 050 shape_items
        "065-i_cups",                # 051 shape_items
        "065-j_cups",                # 052 shape_items
        # missing: Rope, Credit card blank
        "070-a_colored_wood_blocks", # 053 task_items
        "070-b_colored_wood_blocks", # 054 task_items
        "071_nine_hole_peg_test",    # 055 task_items
        "072-a_toy_airplane",        # 056 task_items
        "072-b_toy_airplane",        # 057 task_items
        "072-c_toy_airplane",        # 058 task_items
        "072-d_toy_airplane",        # 059 task_items
        "072-e_toy_airplane",        # 060 task_items
        "073-a_lego_duplo",          # 061 task_items
        "073-b_lego_duplo",          # 062 task_items
        "073-c_lego_duplo",          # 063 task_items
        "073-d_lego_duplo",          # 064 task_items
        "073-e_lego_duplo",          # 065 task_items
        "073-f_lego_duplo",          # 066 task_items
        "073-g_lego_duplo",          # 067 task_items
        "077_rubiks_cube",           # 068 task_items
        "RotatingTable"  ,           # ROTATING TABLE
        # missing: Black t-shirt, Timer, Magazine
    ]
}

def image_rect_color_cb(msg):
	global _rgb
	global _rgb_mat
	try:
            bridge_rgb = CvBridge()
            _rgb_mat = bridge_rgb.imgmsg_to_cv2(msg, msg.encoding)
	    _rgb = True

        except CvBridgeError as cv_bridge_exception:
            rospy.logerr(cv_bridge_exception)
	    _rgb = False

def spawn_furniture(gazebo_name, name, x, y, z, orientation):
    rospy.loginfo('Spawn: {0}'.format(name))
    initial_pose = Pose()
    initial_pose.position.x = x
    initial_pose.position.y = y
    initial_pose.position.z = z
    roll = orientation[0]
    pitch = orientation[1]
    yaw = orientation[2]
    q = tft.quaternion_from_euler(roll, pitch, yaw)
    initial_pose.orientation = Quaternion(q[0], q[1], q[2], q[3])

    model_xml = model_database_template.replace('MODEL_NAME', name)

    gazebo_interface.spawn_sdf_model_client(gazebo_name, model_xml, rospy.get_namespace(),
                                            initial_pose, "", "/gazebo")

def spawn_object(gazebo_name, name, x, y, z, yaw):
    rospy.loginfo('Spawn: {0}'.format(name))
    initial_pose = Pose()
    initial_pose.position.x = x
    initial_pose.position.y = y
    initial_pose.position.z = z
    roll = 0.0
    pitch = 0.0
    yaw = math.pi * (random.random() - 0.5)
    q = tft.quaternion_from_euler(roll, pitch, yaw)
    initial_pose.orientation = Quaternion(q[0], q[1], q[2], q[3])

    path_xml = _path_xml.replace('MODEL_NAME', name)

    with open(path_xml, "r") as f:
        model_xml = f.read()

    model_xml = model_xml.replace('PATH_TO_MODEL', _path_model)

    gazebo_interface.spawn_sdf_model_client(gazebo_name, model_xml, rospy.get_namespace(),
                                            initial_pose, "", "/gazebo")

if __name__ == '__main__':

    #rospy.init_node('spawn_objects')

    #ROS SERVICES
    #RGB image topic
    color_image_sub = rospy.Subscriber('/hsrb/head_rgbd_sensor/rgb/image_rect_color', Image, image_rect_color_cb)

    #Start ObjectFinder client
    #rospy.wait_for_service('/erasers/navigation/object_finder_srv')
    #get_objects_centroid = rospy.ServiceProxy('/erasers/navigation/object_finder_srv', GetObjectsCentroid)

    tf_buffer = tf2_ros.Buffer(rospy.Duration(5.))
    tf2_ros.TransformListener(tf_buffer)

    #SPAWN FURNITURE
    rospy.wait_for_service('/gazebo/delete_model')
    delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
    objs=objects['items']
    print (objs)
    model_name= "073-g_lego_duplo"
    spawn_object(objs[-2], objs[-2], -1.5, 0.5, 1, 0)
    tilt = -0.36
    whole_body.move_to_go()
    whole_body.gaze_point((1,1,0),'map')

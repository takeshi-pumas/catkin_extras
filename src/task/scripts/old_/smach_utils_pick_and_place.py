#!/usr/bin/env python3

import smach
import smach_ros
from geometry_msgs.msg import PoseStamped, Point, PointStamped, Quaternion, TransformStamped, Twist, Pose
from std_srvs.srv import Trigger, TriggerResponse 
import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import rospy 
import numpy as np
import actionlib
from std_srvs.srv import Empty
from sensor_msgs.msg import Image , LaserScan , PointCloud2
import tf
import time
from rospy.exceptions import ROSException
import moveit_commander
import moveit_msgs.msg

from utils.grasp_utils import *
from utils.misc_utils import *
from utils.nav_utils import *

global rgbd, head, wrist, tf_man, gripper, omni_base, wrist, cabeza, voice
global goal, navclient, arm, robot, scene, clear_octo_client, calibrate_wrist


#wbw = moveit_commander.MoveGroupCommander('whole_body_weighted')
rospy.init_node("SMACH_pick_place_cassette")

#----------Moveit instances
moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
arm = moveit_commander.MoveGroupCommander('arm')

arm.set_planning_time(10)
arm.set_num_planning_attempts(25)

#----------Class instances
rgbd        = RGBD()
tf_man      = TF_MANAGER()
gripper     = GRIPPER()
omni_base   = NAVIGATION()
wrist       = WRIST_SENSOR()
cabeza      = GAZE()
brazo       = ARM()
voice       = TALKER()

#----------ROS services
clear_octo_client = rospy.ServiceProxy('/clear_octomap', Empty)
calibrate_wrist = rospy.ServiceProxy('/hsrb/wrist_wrench/readjust_offset',Empty)

#----------Functions

def wait_for_push_hand(time=10):

    start_time = rospy.get_time()
    time = 10
    print('timeout will be ',time,'seconds')
    while (rospy.get_time() - start_time < time):
        torque = wrist.get_torque()
        if np.abs(torque[1]) > 1.0:
            print('Hand pused ready to start')
            voice.talk('Im ready to start')
            return True
            break

    if (rospy.get_time() - start_time >= time):
        print(time, 'secs have elapsed with no hand push')
        return False
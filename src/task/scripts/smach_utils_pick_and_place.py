#!/usr/bin/env python3

import smach
import smach_ros
from geometry_msgs.msg import PoseStamped, Point , PointStamped , Quaternion , TransformStamped , Twist, Pose
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
global clear_octo_client, scene, goal, navclient, arm

rospy.init_node('smach')

#----------Moveit instances
arm = moveit_commander.MoveGroupCommander('whole_body_weighted')

#----------Class instances
rgbd        = RGBD()
tf_man      = TF_MANAGER()
gripper     = GRIPPER()
omni_base   = NAVIGATION()
wrist       = WRIST_SENSOR()
cabeza      = GAZE()
brazo       = ARM()
voice       = TALKER()

#------------------------------------------------------

def wait_for_push_hand(time=10):

    start_time = rospy.get_time()
    time = 10
    print('timeout will be ',time,'seconds')
    while (rospy.get_time() - start_time < time):
        torque = wrist.get_torque()
        if np.abs(torque[1]) > 1.0:
            print('Hand pused ready to start')
            #takeshi_talk_pub.publish(string_to_Voice())
            voice.talk('Im ready to start')
            return True
            break

    if (rospy.get_time() - start_time >= time):
        print(time, 'secs have elapsed with no hand push')
        return False
#
#------------------------------------------------------
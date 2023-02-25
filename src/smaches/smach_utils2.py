#!/usr/bin/env python3

import smach
import smach_ros
from geometry_msgs.msg import PoseStamped, Point , Quaternion , TransformStamped , Twist
from std_srvs.srv import Trigger, TriggerResponse 
import moveit_commander
import moveit_msgs.msg
import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from object_classification.srv import *
from segmentation.srv import *
from ros_whisper_vosk.srv import GetSpeech
#import face_recognition 
import cv2  
import rospy 
import numpy as np
import actionlib
from hmm_navigation.msg import NavigateActionGoal, NavigateAction
from cv_bridge import CvBridge, CvBridgeError
import pandas as pd
from std_srvs.srv import Empty
from sensor_msgs.msg import Image , LaserScan , PointCloud2
import tf
import time

from utils.grasp_utils import *
from utils.misc_utils import *
from utils.nav_utils import *

global listener, broadcaster, tfBuffer, tf_static_broadcaster, scene, rgbd  , head,whole_body,arm,gripper 
global clear_octo_client, goal,navclient,segmentation_server  , tf_man , gaze,omni_base ,speech_recog_server
rospy.init_node('smach')
#head = moveit_commander.MoveGroupCommander('head')
#gripper =  moveit_commander.MoveGroupCommander('gripper')
#whole_body=moveit_commander.MoveGroupCommander('whole_body')
#arm =  moveit_commander.MoveGroupCommander('arm')
#broadcaster = tf.TransformBroadcaster()

tfBuffer = tf2_ros.Buffer()

listener = tf2_ros.TransformListener(tfBuffer)
broadcaster = tf2_ros.TransformBroadcaster()
tf_static_broadcaster = tf2_ros.StaticTransformBroadcaster()
clear_octo_client = rospy.ServiceProxy('/clear_octomap', Empty)
segmentation_server = rospy.ServiceProxy('/segment' , Segmentation)
navclient=actionlib.SimpleActionClient('/navigate', NavigateAction)   ### PUMAS NAV ACTION LIB
scene = moveit_commander.PlanningSceneInterface()
speech_recog_server = rospy.ServiceProxy('/speech_recognition/vosk_service' ,GetSpeech)


#segmentation_server = rospy.ServiceProxy('/segment_2_tf', Trigger) 


#whole_body.set_workspace([-6.0, -6.0, 6.0, 6.0]) 
#df=pd.read_csv('/home/takeshi/Codes/known_locations.txt')
#############################################################################
#navclient=actionlib.SimpleActionClient('/navigate_hmm', NavigateAction)   ### HMM NAV
#navclient = #actionlib.SimpleActionClient('/move_base/move', MoveBaseAction)########TOYOTA NAV
###############################################################################################





tf_man = TF_MANAGER()
gaze = GAZE()
gripper = GRIPPER()
omni_base=OMNIBASE()


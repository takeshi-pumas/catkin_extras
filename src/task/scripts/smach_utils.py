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

from grasp_utils import *

global listener, broadcaster, tfBuffer, tf_static_broadcaster, scene, rgbd  , head,whole_body,arm,gripper 
global clear_octo_client, goal,navclient,segmentation_server ,df , tf_man , gaze ,speech_recog_server
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
#segmentation_server = rospy.ServiceProxy('/segment_2_tf', Trigger) 


#whole_body.set_workspace([-6.0, -6.0, 6.0, 6.0]) 
scene = moveit_commander.PlanningSceneInterface()
df=pd.read_csv('/home/takeshi/Codes/known_locations.txt')
speech_recog_server = rospy.ServiceProxy('/speech_recognition/vosk_service' ,GetSpeech)
#############################################################################
navclient=actionlib.SimpleActionClient('/navigate', NavigateAction)   ### PUMAS NAV ACTION LIB
#navclient=actionlib.SimpleActionClient('/navigate_hmm', NavigateAction)   ### HMM NAV
#navclient = #actionlib.SimpleActionClient('/move_base/move', MoveBaseAction)########TOYOTA NAV
###############################################################################################


base_vel_pub = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=10)



tf_man = TF_MANAGER()
gaze = GAZE()
gripper = GRIPPER()
grasp_base=OMNIBASE()
arm = ARM()

def strmsg_to_float(s):
    cent_map=[]
    for l in s[1:-1].split('['):
        if len (l) !=0:
            x,y,z=l[:-3].split(',')
            cent_map.append((float(x),float(y),float(z)))
    return(np.asarray(cent_map))
        
        
def move_base(goal_x,goal_y,goal_yaw,time_out=10):
    nav_goal= NavigateActionGoal()
    nav_goal.goal.x= goal_x
    nav_goal.goal.y= goal_y
    nav_goal.goal.yaw=goal_yaw
    nav_goal.goal.timeout= time_out

    print (nav_goal)

    # send message to the action server
    navclient.send_goal(nav_goal.goal)


    # wait for the action server to complete the order
    navclient.wait_for_result(timeout=rospy.Duration(time_out))

    # print result of navigation
    action_state = navclient.get_state()
    return navclient.get_state()

def write_tf(pose, q, child_frame , parent_frame='map',time=0):
    t= TransformStamped()
    t.header.stamp = rospy.Time(time)
    t.header.frame_id =parent_frame
    t.child_frame_id =  child_frame
    t.transform.translation.x = pose[0]
    t.transform.translation.y = pose[1]
    t.transform.translation.z = pose[2]
    #q = tf.transformations.quaternion_from_euler(eu[0], eu[1], eu[2])
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]
    return t
def read_tf(t):
    pose=np.asarray((
        t.transform.translation.x,
        t.transform.translation.y,
        t.transform.translation.z
        ))
    quat=np.asarray((
        t.transform.rotation.x,
        t.transform.rotation.y,
        t.transform.rotation.z,
        t.transform.rotation.w
        ))
    
    return pose, quat


def get_current_time_sec():

    return rospy.Time.now().to_sec()


def move_abs(vx,vy,vw, time=0.05):
    start_time = get_current_time_sec() 
    while get_current_time_sec() - start_time < time: 
        twist = Twist()
        twist.linear.x = vx
        twist.linear.y = vy
        twist.angular.z = vw / 180.0 * np.pi  
        base_vel_pub.publish(twist)
def open_gripper():
    target_motor=1
    gripper.set_start_state_to_current_state()
    try:
        gripper.set_joint_value_target({'hand_motor_joint':target_motor})
    except:
        print('OOB')
    succ=gripper.go()
def close_gripper():
    target_motor=0.0
    gripper.set_start_state_to_current_state()
    try:
        gripper.set_joint_value_target({'hand_motor_joint':target_motor})
    except:
        print('OOB')
    succ=gripper.go()


def move_D(target_pose,D):
    target_pose=np.asarray(target_pose)
    robot_pose,quat_r = tf_man.getTF(target_frame='base_link')
    yaw=tf.transformations.euler_from_quaternion(quat_r)[2]
    robot_pose=np.asarray(robot_pose)
    target_rob = target_pose-robot_pose
    goal_pose= target_pose-(target_rob*D/np.linalg.norm(target_rob))
    
    return goal_pose,yaw
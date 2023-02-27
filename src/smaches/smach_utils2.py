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
from face_recog.msg import *
from face_recog.srv import *
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
from cv_bridge import CvBridge, CvBridgeError

from utils.grasp_utils import *
from utils.misc_utils import *
from utils.nav_utils import *

global listener, broadcaster, tfBuffer, tf_static_broadcaster, scene, rgbd  , head,train_new_face
global clear_octo_client, goal,navclient,segmentation_server  , tf_man , gaze,omni_base ,speech_recog_server,bridge
rospy.init_node('smach')
head = moveit_commander.MoveGroupCommander('head')
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
recognize_face = rospy.ServiceProxy('recognize_face', RecognizeFace)
train_new_face = rospy.ServiceProxy('new_face', RecognizeFace)    

rgbd= RGBD()
bridge = CvBridge()
#segmentation_server = rospy.ServiceProxy('/segment_2_tf', Trigger) 


#whole_body.set_workspace([-6.0, -6.0, 6.0, 6.0]) 
#df=pd.read_csv('/home/takeshi/Codes/known_locations.txt')
#############################################################################
#navclient=actionlib.SimpleActionClient('/navigate_hmm', NavigateAction)   ### HMM NAV
#navclient = #actionlib.SimpleActionClient('/move_base/move', MoveBaseAction)########TOYOTA NAV
###############################################################################################
def train_face(image, name):
    req=RecognizeFaceRequest()
    strings=Strings()
    string_msg= String()
    string_msg.data=name
    req.Ids.ids.append(string_msg)

    img_msg=bridge.cv2_to_imgmsg(image)
    req.in_.image_msgs.append(img_msg)
    res=train_new_face(req)
    
    return res.Ids.ids[0].data.split(' ')[0] == 'trained'



def wait_for_face(timeout=10):
    
    rospy.sleep(0.3)
    
    start_time = rospy.get_time()
    strings=Strings()
    string_msg= String()
    string_msg.data='Anyone'
    while rospy.get_time() - start_time < timeout:
        img=rgbd.get_image()  
        req=RecognizeFaceRequest()
        print ('Got  image with shape',img.shape)
        req.Ids.ids.append(string_msg)
        img_msg=bridge.cv2_to_imgmsg(img)
        req.in_.image_msgs.append(img_msg)

        res= recognize_face(req)

        if res.Ids.ids[0].data == 'NO_FACE':
            print ('No face FOund Keep scanning')
        
        else:return res

def wait_for_push_hand(time=10):

    start_time = rospy.get_time()
    time= 10
    print('timeout will be ',time,'seconds')
    while rospy.get_time() - start_time < time:
        torque = wrist.get_torque()
        if np.abs(torque[1])>1.0:
            print(' Hand Pused Ready TO start')
            #takeshi_talk_pub.publish(string_to_Voice())
            talk('Im ready to start')
            return True
            break


    if (rospy.get_time() - start_time >= time):
        print(time, 'secs have elapsed with no hand push')
        return False




tf_man = TF_MANAGER()
gaze = GAZE()
gripper = GRIPPER()
omni_base=OMNIBASE()


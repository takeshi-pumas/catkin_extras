#!/usr/bin/env python3

import smach
import smach_ros
from geometry_msgs.msg import PoseStamped, Point , PointStamped , Quaternion , TransformStamped , Twist
from std_srvs.srv import Trigger, TriggerResponse 
import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from object_classification.srv import *
from segmentation.srv import *
from human_detector.srv import Human_detector ,Human_detectorResponse 
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
from nav_msgs.msg import OccupancyGrid
from hri_msgs.msg import RecognizedSpeech
from rospy.exceptions import ROSException
from vision_msgs.srv import *


from ros_whisper_vosk.srv import SetGrammarVosk

from utils.grasp_utils import *
from utils.misc_utils import *
from utils.nav_utils import *
from utils.know_utils import *

global listener, broadcaster, tfBuffer, tf_static_broadcaster, scene, rgbd, head,train_new_face, wrist, human_detect_server, line_detector, clothes_color
global clear_octo_client, goal,navclient,segmentation_server  , tf_man , omni_base, brazo, speech_recog_server, bridge, map_msg, pix_per_m, analyze_face , arm , set_grammar

rospy.init_node('smach')
# head = moveit_commander.MoveGroupCommander('head')
#gripper =  moveit_commander.MoveGroupCommander('gripper')
#whole_body=moveit_commander.MoveGroupCommander('whole_body')

#broadcaster = tf.TransformBroadcaster()
tfBuffer = tf2_ros.Buffer()

listener = tf2_ros.TransformListener(tfBuffer)
broadcaster = tf2_ros.TransformBroadcaster()
tf_static_broadcaster = tf2_ros.StaticTransformBroadcaster()
# clear_octo_client = rospy.ServiceProxy('/clear_octomap', Empty)   ###OGRASPING OBSTACLE 
human_detect_server = rospy.ServiceProxy('/detect_human' , Human_detector)  ####HUMAN FINDER OPPOSEBASED
# segmentation_server = rospy.ServiceProxy('/segment' , Segmentation)    ##### PLANE SEGMENTATION (PARALEL TO FLOOR)
# navclient=actionlib.SimpleActionClient('/navigate', NavigateAction)   ### PUMAS NAV ACTION LIB
# scene = moveit_commander.PlanningSceneInterface()
speech_recog_server = rospy.ServiceProxy('/speech_recognition/vosk_service' ,GetSpeech)##############SPEECH VOSK RECOG FULL DICT
set_grammar = rospy.ServiceProxy('set_grammar_vosk', SetGrammarVosk)                   ###### Get speech vosk keywords from grammar (function get_keywords)         

recognize_face = rospy.ServiceProxy('recognize_face', RecognizeFace)                    #FACE RECOG
train_new_face = rospy.ServiceProxy('new_face', RecognizeFace)                          #FACE RECOG
analyze_face = rospy.ServiceProxy('analyze_face', RecognizeFace)    ###DEEP FACE ONLY


#map_msg= rospy.wait_for_message('/augmented_map', OccupancyGrid)####WAIT for nav pumas map
#inflated_map= np.asarray(map_msg.data)
#img_map=inflated_map.reshape((map_msg.info.width,map_msg.info.height))
#pix_per_m=map_msg.info.resolution
#contours, hierarchy = cv2.findContours(img_map.astype('uint8'),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
#contoured=cv2.drawContours(img_map.astype('uint8'), contours, 1, (255,255,255), 1)

rgbd= RGBD()
bridge = CvBridge()
#segmentation_server = rospy.ServiceProxy('/segment_2_tf', Trigger) 
tf_man = TF_MANAGER()
gripper = GRIPPER()
omni_base=NAVIGATION()
wrist= WRIST_SENSOR()
head = GAZE()
brazo = ARM()
line_detector = LineDetector()
# arm =  moveit_commander.MoveGroupCommander('arm')


#------------------------------------------------------
def get_robot_px():
    trans, rot=tf_man.getTF('base_link')
    robot=np.asarray(trans[:2])
    print (trans)
    return np.asarray((robot/pix_per_m).round(),dtype='int')

#------------------------------------------------------
def check_point_map(x,y):

    # Quiero saber punto 0,-7 m esta libre?
    ##meters, map (not pixels)
    #x,y=0.2,8
    #px=check_point_map(x,y)  # px is closest safe goal
    #px*pix_per_m
    xrob,yrob=get_robot_px()
    safe_xy=np.asarray((x,-1*y))
    safe_xy_px=point_to_px (x,y)
    delta_px=  safe_xy_px-np.asarray((xrob,yrob))
    delta_px= delta_px / np.linalg.norm(delta_px).round()
    delta_px=np.round(delta_px*10)
    newxy=np.zeros(2)
    
    for i in range(9):
        if (contoured[1024+ safe_xy_px[1].astype('int'),1024-safe_xy_px[0].astype('int')]!=0):   ## axis are twisted cause y sign
                    print ('not safe at', safe_xy_px , safe_xy_px*pix_per_m)
                    
                    xrob,yrob=get_robot_px()
                    delta_px=  safe_xy_px-np.asarray((xrob,yrob))
                    delta_px= delta_px / np.linalg.norm(delta_px).round()
                    delta_px=np.round(delta_px*10)


                    newxy[0]=safe_xy_px[0]-delta_px[0].astype('int')
                    newxy[1]=safe_xy_px[1]-delta_px[1].astype('int')
                    safe_xy_px=newxy
        else:
                print ('safe at', safe_xy_px, safe_xy_px*pix_per_m)
                return safe_xy_px
    return safe_xy_px
    
#------------------------------------------------------
def point_to_px(x,y):
    safe_xy=np.asarray((x,y))
    return np.round(safe_xy/pix_per_m).astype('int')
def px_to_point(px,py):
    return np.asarray((px,py))*pix_per_m

#------------------------------------------------------
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

    #------------------------------------------------------
def wait_for_face(timeout=10 , name=''):
    
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


        #NO FACE FOUND
        if res.Ids.ids[0].data == 'NO_FACE':
            print ('No face FOund Keep scanning')
            
            return None, None
        #AT LEAST ONE FACE FOUND
        else:
            print('at least one face found')
            ds_to_faces=[]
            for i , idface in enumerate(res.Ids.ids):
                print (i,idface.data)
                ds_to_faces.append(res.Ds.data[i])    ##
                if (idface.data)==name :
                    new_res= RecognizeFaceResponse()
                    new_res.Ds.data= res.Ds.data[i]
                    new_res.Angs.data= res.Angs.data[i:i+4]
                    new_res.Ids.ids=res.Ids.ids[i].data
                    print('return res,img',new_res)
                    print ('hit',idface.data, 'at' , res.Ds.data[i]  , 'meters')
                    ds_to_faces=[]
                    return new_res , img

            if len (ds_to_faces)!=0:
                i=np.argmin(ds_to_faces)
                new_res= RecognizeFaceResponse()
                new_res.Ds.data= res.Ds.data[i]
                new_res.Angs.data= res.Angs.data[i:i+4]
                new_res.Ids.ids=res.Ids.ids[i].data
                print('return res,img',new_res)
                ds_to_faces=[]
                return new_res , img





#------------------------------------------------------

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
#
##------------------------------------------------------
def analyze_face_from_image(cv2_img,name=''):   
    #USING DEEP FACE SERVICE
    # FIND SOME CHARACTERISTICS FROM A FACE IMAGE
    req=RecognizeFaceRequest()
    strings=Strings()
    string_msg= String()
    string_msg.data='analyze '
    req.Ids.ids.append(string_msg)
    img_msg=bridge.cv2_to_imgmsg(cv2_img)
    req.in_.image_msgs.append(img_msg)
    #res=recognize(req)

    #service call
    res = analyze_face(req)
    results=[]
    for chars in res.Ids.ids:
        results.append(chars.data)
    if (len(results)==1):return ''  ###NO FACE
    #name= 'Jack'
    pronoun='she'
    gender = results[0]
    age = results[-1]
    state = results[2]
    race = results[1]
    if gender=='Man':pronoun='he'
    takeshi_line = f'{name} has arrived... {pronoun} is a {gender}... I believe {pronoun}.' 
    takeshi_line += f'is  around  {age} years old... I would say he is a bit  {state}.'
    takeshi_line += f'And I might guess {pronoun} is of {race} descent.'
    return takeshi_line
#------------------------------------------------------
def analyze_face_background(img, name=" "):
    name_pub = rospy.Publisher('/name_face', String, queue_size=10)
    img_pub = rospy.Publisher('/image_to_analyze', Image, queue_size=10)
    str_msg.data = name
    name_pub.publish(str_msg)
    img_msg=bridge.cv2_to_imgmsg(img)
    img_pub.publish(img_msg)
    str_msg = String()



#------------------------------------------------------
def bbox_3d_mean(points,boundRect):
    #Gets the mean of the Points from the pincloud enclosed in the bound Rect of the RGBD image

    xyz=[]
    xyz_n=points[['x','y','z']][boundRect[0]:boundRect[2],boundRect[3]:boundRect[1]]
    for i in range(xyz_n.shape[0]):
        for j in range(xyz_n.shape[1]):
            if ~np.isnan(xyz_n['x'][i,j]) and ~np.isnan(xyz_n['y'][i,j]) and ~np.isnan(xyz_n['z'][i,j]):
                xyz.append(np.asarray([xyz_n['x'][i,j],xyz_n['y'][i,j],xyz_n['z'][i,j]]))
                
    if len(xyz)!=0:
        return np.asarray(xyz).mean(axis=0)
    else:
        return np.ones(3)

#------------------------------------------------------
def detect_human_to_tf():
    humanpose=human_detect_server.call()
    print (humanpose)
    if (np.asarray((humanpose.x,humanpose.y,humanpose.z)).all()== np.zeros(3).all()):
        print (np.asarray((humanpose.x,humanpose.y,humanpose.z)))
        return False
    else:
        tf_man.pub_static_tf(np.asarray((humanpose.x,humanpose.x,humanpose.z)),point_name='human', ref='head_rgbd_sensor_link')
        succ=tf_man.change_ref_frame_tf('human')
        return succ


def get_keywords_speech(timeout=5):
    pub = rospy.Publisher('/talk_now', String, queue_size=10)
    rospy.sleep(1.0)
    msg = String()
    msg.data='start'
    pub.publish(msg)
    try:
        
        msg = rospy.wait_for_message('/speech_recognition/final_result', String, timeout)
        result = msg.data
        pub.publish(String())
        rospy.sleep(1.0)
        return result
            
    except ROSException:
        rospy.loginfo('timeout')
        pub.publish(String())
        return 'timeout'

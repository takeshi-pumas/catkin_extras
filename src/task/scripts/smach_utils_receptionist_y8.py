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
import math
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
from std_msgs.msg import String, Bool
import random
import rospkg
import yaml
import pandas as pd
rospack = rospkg.RosPack()
from ros_whisper_vosk.srv import SetGrammarVosk

from utils import grasp_utils, misc_utils, nav_utils, receptionist_knowledge

global listener, broadcaster, tfBuffer, tf_static_broadcaster, scene, rgbd, head,train_new_face, wrist, human_detect_server, line_detector, clothes_color
global clear_octo_client, goal,navclient,segmentation_server  , tf_man , omni_base, brazo, speech_recog_server, bridge, map_msg, pix_per_m, analyze_face , arm , set_grammar
#   arm =  moveit_commander.MoveGroupCommander('arm')
rospy.init_node('smach_receptionist')
# TF2_ROS setup
tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)
broadcaster = tf2_ros.TransformBroadcaster()
tf_static_broadcaster = tf2_ros.StaticTransformBroadcaster()

# Service callers
human_detect_server = rospy.ServiceProxy('/detect_human' , Human_detector)  ####HUMAN FINDER OPPOSEBASED
speech_recog_server = rospy.ServiceProxy('/speech_recognition/vosk_service' ,GetSpeech)##############SPEECH VOSK RECOG FULL DICT
set_grammar = rospy.ServiceProxy('set_grammar_vosk', SetGrammarVosk)                   ###### Get speech vosk keywords from grammar (function get_keywords)         
recognize_face = rospy.ServiceProxy('recognize_face', RecognizeFace)                    #FACE RECOG
train_new_face = rospy.ServiceProxy('new_face', RecognizeFace)                          #FACE RECOG
analyze_face = rospy.ServiceProxy('analyze_face', RecognizeFace)    ###DEEP FACE ONLY
classify_client_yolo = rospy.ServiceProxy('classify_yolov8', Classify_yolo_receptionist) #beverage recognition
segment_service = rospy.ServiceProxy("segment_region", SegmentRegion) # Beverage area segmentation


enable_mic_pub = rospy.Publisher('/talk_now', Bool, queue_size=10)

# Utils
rgbd= misc_utils.RGBD()
bridge = CvBridge()
tf_man = misc_utils.TF_MANAGER()
gripper = grasp_utils.GRIPPER()
omni_base = nav_utils.NAVIGATION()
wrist= grasp_utils.WRIST_SENSOR()
head = grasp_utils.GAZE()
brazo = grasp_utils.ARM()
line_detector = misc_utils.LineDetector()
voice = misc_utils.TALKER()
party = receptionist_knowledge.RECEPTIONIST()

# Functions
def places_2_tf():
    places, locs = party.get_places_location()
    for place, loc in zip(places, locs):
        print(place, loc)
        pos = [loc[0], loc[1], 0.85]
        rot = tf.transformations.quaternion_from_euler(0.0, 0.0, loc[2])
        tf_man.pub_static_tf(pos=pos, rot=rot, point_name=place)
        rospy.sleep(0.6)
        tf_face = place.replace('_', '_face')
        tf_man.pub_static_tf(pos=[1.0, 0, 0], rot=rot, point_name=tf_face, ref=place)
        rospy.sleep(0.6)
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
        img_msg=bridge.cv2_to_imgmsg(img[:,150:-150])
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
def faces_2_tf(res):
    
    for place, loc in zip(places, locs):
        print(place, loc)
        pos = [loc[0], loc[1], 0.85]
        rot = tf.transformations.quaternion_from_euler(0.0, 0.0, loc[2])
        tf_man.pub_static_tf(pos=pos, rot=rot, point_name=place)
        rospy.sleep(0.6)
        tf_face = place.replace('_', '_face')
        tf_man.pub_static_tf(pos=[1.0, 0, 0], rot=rot, point_name=tf_face, ref=place)
        rospy.sleep(0.6)
#------------------------------------------------------

def wait_for_push_hand(time=10):

    start_time = rospy.get_time()
    time= 10
    print('timeout will be ',time,'seconds')
    while rospy.get_time() - start_time < time and not rospy.is_shutdown():
        torque = wrist.get_torque()
        if np.abs(torque[1])>1.0:
            print(' Hand Pused Ready To Start')
            #takeshi_talk_pub.publish(string_to_Voice())
            #talk('Im ready to start')
            return True

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
    str_msg = String()
    str_msg.data = name
    rospy.sleep(0.5)
    name_pub.publish(str_msg)
    img_msg = bridge.cv2_to_imgmsg(img)
    img_pub.publish(img_msg)

#------------------------------------------------------
def get_favorite_drink_location_yolo(favorite_drink):
    bridge = CvBridge()
    # Convert image to ROS format
    pointcloud_msg = rospy.wait_for_message("/hsrb/head_rgbd_sensor/depth_registered/rectified_points", PointCloud2)
    img_msg = rospy.wait_for_message('/hsrb/head_rgbd_sensor/rgb/image_raw', Image, timeout=5)
    img = bridge.imgmsg_to_cv2(img_msg,"bgr8")
    if pointcloud_msg is None:
        rospy.logerr("No se recibió la nube de puntos.")
        return
    rospy.loginfo("Nube de puntos recibida. Enviando solicitud de segmentación...")
    # Nombre de la región a segmentar (ajustar según el YAML)
    region_name = "beverage_area"
    rospy.wait_for_service("segment_region")
    try:
        request = SegmentRegionRequest(pointcloud=pointcloud_msg, region_name=region_name)
        response = segment_service(request)

        if response.success:
            bridge = CvBridge()
            mask = bridge.imgmsg_to_cv2(response.mask, "mono8")
            # **Aplicar operaciones morfológicas para reducir ruido**
            kernel = np.ones((13, 13), np.uint8)  # Define un kernel de 5x5 (ajustable)
            mask = cv2.dilate(mask, kernel, iterations=4)  # **Rellena huecos**
            #mask = cv2.erode(mask, kernel, iterations=1)  # **Reduce pequeños artefactos**
            segment_img = cv2.bitwise_and(img, img, mask=mask)
            cv2.imwrite("img_debug.png",segment_img)
        else:
            rospy.logwarn("Error en segmentación: " + response.message)
    except rospy.ServiceException as e:
        rospy.logerr("Error llamando al servicio: %s" % e)
    print("Message Received")
    ros_image=bridge.cv2_to_imgmsg(segment_img,encoding="bgr8")
    prompt_msg = String()
    prompt_msg.data = favorite_drink

    rospy.wait_for_service('classify_yolov8')
    try:
        response = classify_client_yolo(ros_image, prompt_msg)
        print("Result:", response.result.data,"for drink:",favorite_drink)
        if response.result.data == "not found":
            res = False
        else: 
            res = True
        return res,response.result.data

    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")
        return False,e
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
    enabled_msg = Bool()
    enabled_msg.data = True
    enable_mic_pub.publish(enabled_msg)
    rospy.sleep(0.5)
    try:
        msg = rospy.wait_for_message('/speech_recognition/final_result', String, timeout)
        result = msg.data
        enabled_msg = Bool()
        enabled_msg.data = False
        enable_mic_pub.publish(enabled_msg)
        #rospy.sleep(1.0)
        return result
            
    except ROSException:
        rospy.loginfo('timeout')
        enabled_msg = Bool()
        enabled_msg.data = False
        enable_mic_pub.publish(enabled_msg)
        return 'timeout'

def match_speech(speech, to_match):
    for element in to_match:
        if element in speech:
            return True
    return False

#------------------------------------------------------
def new_move_D_to(tf_name='placing_area',d_x=15 , timeout=30.0):

    timeout = rospy.Time.now().to_sec() + timeout
    succ = False            
    i=0
    while (timeout >= rospy.Time.now().to_sec()) and not succ and not rospy.is_shutdown():
        
        _,rot=tf_man.getTF('base_link')    
        robot_yaw=tf.transformations.euler_from_quaternion(rot)[2]
        pose,_= tf_man.getTF("base_link",ref_frame=tf_name)
        target_yaw = math.atan2(pose[1], pose[0])+np.pi
        delta_th=   target_yaw-robot_yaw
        delta_th = (delta_th + np.pi) % (2 * np.pi) - np.pi
        i+=1                
        eX = np.linalg.norm((pose[0:2]))
        eX+= -d_x  #x offest
        velX= eX 
        if abs(delta_th)>=0.1:velX=0
        succ =  eX <= 0.1  and abs(delta_th)<=0.1
        corr_velX = max(min(velX, 0.051), -0.051)
        if i >=10:
            print("error_D: {:.2f}, , delta_th {:.2f}, target obj frame {}".format(eX,  delta_th,tf_name))
            i=0
        omni_base.tiny_move( velX=corr_velX,velY=0, velT=delta_th,std_time=0.2, MAX_VEL=0.3) 
    return succ



## PARAM  FILE
def read_yaml(known_locations_file = '/known_locations.yaml'):
    
    file_path = rospack.get_path('config_files')  + known_locations_file

    with open(file_path, 'r') as file:
        content = yaml.safe_load(file)
    return content

def yaml_to_df():
    con = read_yaml('/known_locations.yaml')
    values=[]
    locations=[]
    for c in con:
        locations.append(c)

        for i in range(len(con[c])):
            values.append(list(con[c][i].values())[0])

    data=np.asarray(values).reshape((int(len(values)/7),7))    #x , y ,theta  ,quat   since z always 0
    df= pd.DataFrame( data)
    df.columns=['x','y','th','qx','qy','qz','qw']
    df['child_id_frame']=locations
    return df
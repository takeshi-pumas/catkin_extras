#!/usr/bin/env python3
from scipy.spatial import KDTree
import cv2  
import rospy 
import numpy as np
import pandas as pd
import smach
import smach_ros
import actionlib
import rospkg
import yaml
import tf
import time
import moveit_commander
import moveit_msgs.msg
import tf2_ros
import logging 
from os import path
from glob import glob
import ros_numpy
from geometry_msgs.msg import PoseStamped, Point  , Quaternion , TransformStamped , Twist
from tf2_geometry_msgs import PointStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse 
from sensor_msgs.msg import Image , LaserScan  , PointCloud2
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from object_classification.srv import *
from segmentation.srv import *
from human_detector.srv import Human_detector  ,Human_detectorRequest 
from human_detector.srv import Point_detector ,Point_detectorRequest
from ros_whisper_vosk.srv import GetSpeech
from object_classification.srv import *
from face_recog.msg import *
from face_recog.srv import *
#import face_recognition 
from hmm_navigation.msg import NavigateActionGoal, NavigateAction
from cv_bridge import CvBridge, CvBridgeError
from std_srvs.srv import Empty
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import OccupancyGrid
from hri_msgs.msg import RecognizedSpeech
from rospy.exceptions import ROSException
from vision_msgs.srv import *
#from act_recog.srv import Recognize,RecognizeResponse,RecognizeRequest
from ros_whisper_vosk.srv import SetGrammarVosk

from action_server.msg import FollowActionGoal ,  FollowAction

from utils.grasp_utils import *
from utils.misc_utils import *
from utils.nav_utils import *
#from utils.know_utils import *

global listener, broadcaster, tfBuffer, tf_static_broadcaster, scene, rgbd, head,train_new_face, wrist, human_detect_server, line_detector, clothes_color , head_mvit
global clear_octo_client, goal,navclient,segmentation_server  , tf_man , omni_base, brazo, speech_recog_server, bridge, map_msg, pix_per_m, analyze_face , arm , set_grammar
global recognize_action , classify_client,pointing_detect_server ,placing_finder_server
rospy.init_node('smach', anonymous=True)
logger = logging.getLogger('rosout')
logger.setLevel(logging.ERROR)
#head_mvit = moveit_commander.MoveGroupCommander('head')
#gripper =  moveit_commander.MoveGroupCommander('gripper')
#whole_body=moveit_commander.MoveGroupCommander('whole_body')

#broadcaster = tf.TransformBroadcaster()
#arm =  moveit_commander.MoveGroupCommander('arm')
tfBuffer = tf2_ros.Buffer()

listener = tf2_ros.TransformListener(tfBuffer)
broadcaster = tf2_ros.TransformBroadcaster()
tf_static_broadcaster = tf2_ros.StaticTransformBroadcaster()
clear_octo_client = rospy.ServiceProxy('/clear_octomap', Empty)   ###GRASPING OBSTACLE 
human_detect_server = rospy.ServiceProxy('/detect_human' , Human_detector)  ####HUMAN FINDER OPPOSEBASED
pointing_detect_server = rospy.ServiceProxy('/detect_pointing' , Point_detector)  ####HUMAN FINDER OPPOSEBASED
segmentation_server = rospy.ServiceProxy('/segment' , Segmentation)    ##### PLANE SEGMENTATION (PARALEL TO FLOOR)
placing_finder_server = rospy.ServiceProxy('/placing_finder' , Segmentation)### WHERE TO PLACE THINGS IN SHELVES
navclient=actionlib.SimpleActionClient('/navigate', NavigateAction)   ### PUMAS NAV ACTION LIB

# scene = moveit_commander.PlanningSceneInterface()
speech_recog_server = rospy.ServiceProxy('/speech_recognition/vosk_service' ,GetSpeech)##############SPEECH VOSK RECOG FULL DICT
set_grammar = rospy.ServiceProxy('set_grammar_vosk', SetGrammarVosk)                   ###### Get speech vosk keywords from grammar (function get_keywords)         
recognize_face = rospy.ServiceProxy('recognize_face', RecognizeFace)                    #FACE RECOG
train_new_face = rospy.ServiceProxy('new_face', RecognizeFace)                          #FACE RECOG
analyze_face = rospy.ServiceProxy('analyze_face', RecognizeFace)    ###DEEP FACE ONLY
#recognize_action = rospy.ServiceProxy('recognize_act', Recognize) 
classify_client = rospy.ServiceProxy('/classify', Classify)
classify_clnt_stickler = rospy.ServiceProxy('/classifystick', Classify)



####################################################################
map_msg= rospy.wait_for_message('/augmented_map', OccupancyGrid , 20)####WAIT for nav pumas map .. 
inflated_map= np.asarray(map_msg.data)
img_map=inflated_map.reshape((map_msg.info.width,map_msg.info.height))
pix_per_m=map_msg.info.resolution
contours, hierarchy = cv2.findContours(img_map.astype('uint8'),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
contoured=cv2.drawContours(img_map.astype('uint8'), contours, 1, (255,255,255), 1)

####################################################################3


rgbd= RGBD()
bridge = CvBridge()
#segmentation_server = rospy.ServiceProxy('/segment_2_tf', Trigger) 
tf_man = TF_MANAGER()
gripper = GRIPPER()
omni_base=OMNIBASE()        #  NAV ACTION
#omni_base=NAVIGATION()     #  nav UTILS
wrist= WRIST_SENSOR()
head = GAZE()
brazo = ARM()
line_detector = LineDetector()
# arm =  moveit_commander.MoveGroupCommander('arm')
#------------------------------------------------------

def call_yolo_service(height = -1):
    request = segmentation_server.request_class() 
    request.height.data = height

    #head.set_joint_values([ 0.1, -0.5])
    response = segmentation_server.call(request)
    succ = seg_res_tf(response)
    print (f'heights{response.heights.data}, widths {response.widths.data}')
    #img = bridge.imgmsg_to_cv2(response.im_out.image_msgs[0])
    #cv2.imshow('our of res'  , img)
    return response

#------------------------------------------------------
def detect_object_yolo(object_name,res):
    # find object_name in the response message from object_classification service (Yolo)
    objs=[]
    poses=[]
    for i,name in enumerate(res.names):
        if res.poses[i].position.x is not np.nan :
            objs.append(name.data[4:])
            poses.append(res.poses[i])
        if name.data[4:]==object_name:return res.poses[i]
    if object_name=='all':
        print (objs, poses)
        return objs , poses
    return []

#------------------------------------------------------
def seg_res_tf(res):
    # Extract pose information from segmentation response an publish a tf... 
    # No rot is tf with pose relating to map  zero angles (robot facing)
    # the object_number tf is the PCA axis  orientation
    origin_map_img=[round(img_map.shape[0]*0.5) ,round(img_map.shape[1]*0.5)]   
    
    #brazo.set_named_target('go')
    if len(res.poses.data)==0:
        print('no objs')
        return False
    else:
        
        poses=np.asarray(res.poses.data)
        quats=np.asarray(res.quats.data)
        poses=poses.reshape((int(len(poses)/3) ,3     )      )  
        quats=quats.reshape((int(len(quats)/4) ,4     )      )  
        num_objs=len(poses)
        print(f'{num_objs} found')

        for i,cent in enumerate(poses):
            x,y,z=cent
            axis=[0,0,1]
            angle = tf.transformations.euler_from_quaternion(quats[i])[0]
            rotation_quaternion = tf.transformations.quaternion_about_axis(angle, axis)
            point_name=f'object_{i}'
            print (f'{point_name} found at {cent}')
            # tf_man.pub_tf(pos=cent, rot =[0,0,0,1], point_name=point_name+'_norot', ref='map')## which object to choose   #TODO
            # succ=tf_man.pub_tf(pos=cent, rot =rotation_quaternion, point_name=point_name, ref='map')## which object to choose   #TODO
            tf_man.pub_static_tf(pos=cent, rot =[0,0,0,1], point_name=point_name+'_norot', ref='map')## which object to choose   #TODO
            succ=tf_man.pub_static_tf(pos=cent, rot =rotation_quaternion, point_name=point_name, ref='map')## which object to choose   #TODO
            rospy.sleep(0.5)                                                                        
            pose,_= tf_man.getTF(point_name)
            print (f'Occupancy map at point object {i}-> pixels ',origin_map_img[1]+ round(pose[1]/pix_per_m),origin_map_img[0]+ round(pose[0]/pix_per_m), img_map[origin_map_img[1]+ round(pose[1]/pix_per_m),origin_map_img[0]+ round(pose[0]/pix_per_m)])
            ## Pixels from augmented image map server published map image
            if img_map[origin_map_img[1]+ round(pose[1]/pix_per_m),origin_map_img[0]+ round(pose[0]/pix_per_m)]!=0:#### Yes axes seem to be "flipped" !=0:
                print ('reject point suggested ( for floor), most likely part of arena, occupied inflated map')
                #tf_man.pub_static_tf(pos=[0,0,0], point_name=point_name, ref='head_rgbd_sensor_rgb_frame')
                #num_objs-=1
            print (f"object found at map coords.{pose} ")
    return succ

#------------------------------------------------------
def seg_res_tf_pointing(res):
    # Extract pose information from segmentation response an publish a tf... 
    # No rot is tf with pose relating to map  zero angles (robot facing)
    # the object_number tf is the PCA axis  orientation
    origin_map_img=[round(img_map.shape[0]*0.5) ,round(img_map.shape[1]*0.5)]   
    
    #brazo.set_named_target('go')
    if len(res.poses.data)==0:
        print('no objs')
        return False
    else:
        
        poses=np.asarray(res.poses.data)
        quats=np.asarray(res.quats.data)
        poses=poses.reshape((int(len(poses)/3) ,3     )      )  
        quats=quats.reshape((int(len(quats)/4) ,4     )      )  
        num_objs=len(poses)
        print(f'{num_objs} found')

        pointPose,_= tf_man.getTF('pointing_')
        rospy.sleep(0.8)
        print(f'Point pose head: {pointPose}')
        dist=10000
        for i,cent in enumerate(poses):
            print(f'Objeto {i} de la lista con coordenada X:{cent[0]},Y:{cent[1]},Z:{cent[2]}')
            if abs(np.linalg.norm(pointPose-cent)) < dist:
                objIndex = i
                dist = abs(np.linalg.norm(pointPose-cent))
                print(f'Distancia menor, objeto {i} respecto a pointing_ : {dist}')

        axis=[0,0,1]
        angle = tf.transformations.euler_from_quaternion(quats[objIndex])[0]
        rotation_quaternion = tf.transformations.quaternion_about_axis(angle, axis)
        point_name=f'object_0'
        tf_man.pub_static_tf(pos=poses[objIndex], rot =[0,0,0,1], point_name=point_name+'_norot', ref='map')## which object to choose   #TODO
        succ=tf_man.pub_static_tf(pos=poses[objIndex], rot =rotation_quaternion, point_name=point_name, ref='map')## which object to choose   #TODO
        rospy.sleep(0.5)                                                                        
        pose,_= tf_man.getTF(point_name)
        print (f'Occupancy map at point object {i}-> pixels ',origin_map_img[1]+ round(pose[1]/pix_per_m),origin_map_img[0]+ round(poses[objIndex][0]/pix_per_m), img_map[origin_map_img[1]+ round(poses[objIndex][1]/pix_per_m),origin_map_img[0]+ round(poses[objIndex][0]/pix_per_m)])
        ## Pixels from augmented image map server published map image
        if img_map[origin_map_img[1]+ round(poses[objIndex][1]/pix_per_m),origin_map_img[0]+ round(poses[objIndex][0]/pix_per_m)]!=0:#### Yes axes seem to be "flipped" !=0:
            print ('reject point suggested ( for floor), most likely part of arena, occupied inflated map')
            #tf_man.pub_static_tf(pos=[0,0,0], point_name=point_name, ref='head_rgbd_sensor_rgb_frame')
            #num_objs-=1
        print (f"object found at map coords.{pose} ")
        
    return succ

#------------------------------------------------------
def find_placing_area (plane_height=-1):
    #head.set_joint_values([-1.5,-0.65])
    #rospy.sleep(0.5)
    for i in range (4):
        request= segmentation_server.request_class()    
        request.height.data=plane_height+0.01*i  #MID SHELF FOR PLACING 
        print ('#############Finding placing in plane####################',request.height.data)
        res=placing_finder_server.call(request)
        #succ=seg_res_tf(res)
        print (f'Placing Area at {res.poses.data}')    
        if len(res.poses.data)!=0:
            tf_man.pub_static_tf(pos=[res.poses.data[0], res.poses.data[1],res.poses.data[2]], rot =[0,0,0,1], point_name='placing_area')
            return True    
    return False    

#------------------------------------------------------
def get_robot_px():
    trans, rot=tf_man.getTF('base_link')
    robot=np.asarray(trans[:2])
    
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

#------------------------------------------------------
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
#def wait_for_face(timeout=10):
#    
#    rospy.sleep(0.3)
#    
#    start_time = rospy.get_time()
#    strings=Strings()
#    string_msg= String()
#    string_msg.data='Anyone'
#    while rospy.get_time() - start_time < timeout:
#        img=rgbd.get_image()  
#        req=RecognizeFaceRequest()
#        print ('Got  image with shape',img.shape)
#        req.Ids.ids.append(string_msg)
#        img_msg=bridge.cv2_to_imgmsg(img)
#        req.in_.image_msgs.append(img_msg)
#
#        res= recognize_face(req)
#
#        if res.Ids.ids[0].data == 'NO_FACE':
#            print ('No face FOund Keep scanning')
#            return None, None
#        
#        else:return res , img
##------------------------------------------------------
def hand_grasp_D(tf_name='placing_area', THRESHOLD=0.03,timeout=10.0):
    
    timeout = rospy.Time.now().to_sec() + timeout
    succ=False
    THRESHOLD = 0.03
    _, rot  = tf_man.getTF("base_link",ref_frame='map')#ORIGINAL ROTATION TO KEEP DURING APPROACH
    original_rot=tf.transformations.euler_from_quaternion(rot)[2]
    while (timeout >= rospy.Time.now().to_sec()) and not succ and not rospy.is_shutdown():
        trans,_ = tf_man.getTF(target_frame='placing_area', ref_frame='hand_palm_link')
        _, rot  = tf_man.getTF("base_link",ref_frame='map')
        if type(trans) is not bool:
            _, eY, eX = trans
            if abs(eY) < THRESHOLD:
                eY = 0
            if abs(eX) < THRESHOLD:
                eX = 0
            eT= tf.transformations.euler_from_quaternion(rot)[2] - original_rot #Original             
            eT = (eT + np.pi) % (2 * np.pi) - np.pi            
            print("error: {:.2f}, {:.2f}, angle {:.2f}, target obj frame placing area".format(eX, eY , eT))
            if eT > np.pi: eT=-2*np.pi+eT
            if eT < -np.pi: eT= 2*np.pi+eT
            if abs(eT) < 0.05:eT=0
            
            if eX >0: velX = max( 0.001,eX)
            if eX <=0: velX = min(-0.001,eX)
            if eY >0: velY = max( 0.001,eY)
            if eY <=0: velY = min(-0.001,eY)
            print("error: {:.2f}, {:.2f}, angle {:.2f}, target obj frame placing area".format(eX, eY , eT))
            succ =  eX == 0 and eY == 0 and eT==0            
            
            
            
            omni_base.tiny_move(velX=0.25*velX, velY=-1*velY , std_time=0.2, MAX_VEL=0.1) 
    return succ
            
##------------------------------------------------------

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
def read_yaml(known_locations_file='/known_locations.yaml'):
    rospack = rospkg.RosPack()
    file_path = rospack.get_path('config_files') + known_locations_file
    #print("FILE PATH",file_path)
    with open(file_path, 'r') as file:
        content = yaml.safe_load(file)
    return content

#----------------------------------------------------------
def yaml_to_df(known_locations_file='/known_locations.yaml'):
    con = read_yaml(known_locations_file)
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

#------------------------------------------------------
def read_tf(t):
    # trasnform message to np arrays
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


#------------------------------------------------------
def detect_human_to_tf(dist = 6,remove_bkg = True):
    req = Human_detectorRequest()
    req.dist = dist
    req.removeBKG = remove_bkg
    humanpose=human_detect_server(req)
    #print ("humanpose",humanpose)
    if (np.asarray((humanpose.x,humanpose.y,humanpose.z)).all()== np.zeros(3).all()):
        #print ("ASARRAY",np.asarray((humanpose.x,humanpose.y,humanpose.z)))
        return False
    else:
        tf_man.pub_static_tf(np.asarray((humanpose.x,humanpose.y,humanpose.z)),point_name='human', ref='head_rgbd_sensor_link')
        rospy.sleep(0.5)
        succ=tf_man.change_ref_frame_tf('human')
        rospy.sleep(0.5)
        #print("SUCC?", succ)
        return succ

#------------------------------------------------------
def base_grasp_D(tf_name,d_x=0.66,d_y=-0.1,timeout=1.0):
    timeout = rospy.Time.now().to_sec() + timeout
    rob_pos,rot=tf_man.getTF('base_link')    
    original_rot=tf.transformations.euler_from_quaternion(rot)[2]
    succ = False 
    target_object= tf_name        
    i=0
    while (timeout >= rospy.Time.now().to_sec()) and not succ and not rospy.is_shutdown():
        i+=1
        _,rot= tf_man.getTF("base_link",ref_frame='map')
        trans,_=tf_man.getTF(target_object,ref_frame="base_link")
        #trans
        eX, eY, eZ = trans
        eX+= -d_x  #x offest
        eY+= -d_y #y Offset
        eT= tf.transformations.euler_from_quaternion(rot)[2] - original_rot #Original 
        
        rospy.loginfo("error: {:.2f}, {:.2f}, angle {:.2f}, target obj frame {}".format(eX, eY , eT,target_object))
        #if eT > np.pi: eT=-2*np.pi+eT    #  angle error constricted to values between pi and -pi  
        #if eT < -np.pi: eT= 2*np.pi+eT   
        eT = (eT + np.pi) % (2 * np.pi) - np.pi

        X, Y, Z = trans
        rospy.loginfo("Pose: {:.2f}, {:.2f}, angle {:.2f}, target obj frame {}".format(X, Y , eT,target_object))
        if abs(eX) <=0.05 :
            print ('here')
            eX = 0
        if abs(eY) <=0.05  :
            eY = 0
        if abs(eT   ) < 0.03:
            eT = 0
        succ =  eX == 0 and eY == 0 and eT==0         
        velX=0.2*eX
        velY=0.3*eY
        ############################################
        if   velX <= 0: corr_velX= min (-0.0051,velX)
        elif velX >  0: corr_velX= max ( 0.0051,velX)
        if   velY <= 0: corr_velY= min (-0.0051,velY)
        elif velY >  0: corr_velY= max ( 0.005,velY)
        #############################################
        if i %10 ==0 :
            print("Pose: {:.2f}, {:.2f}, angle {:.2f}, target obj frame {}".format(X, Y , eT,target_object))
            i=0
        omni_base.tiny_move( velX=corr_velX,velY=corr_velY, velT=-eT,std_time=0.2, MAX_VEL=0.3) 

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

#------------------------------------------------------

def line_up_TF(tf_name='placing_area', timeout=30.0):
    pose,rot= tf_man.getTF("base_link",ref_frame=tf_name)
    delta_th=tf.transformations.euler_from_quaternion(rot)[2]
    print (pose[1], delta_th)
    timeout = rospy.Time.now().to_sec() + timeout
    active= True
    while (active and timeout >= rospy.Time.now().to_sec()) and not rospy.is_shutdown():
        
        if (abs(pose[1])<0.01 and abs(delta_th)<0.05 ): active = False
        pose,rot= tf_man.getTF("base_link",ref_frame=tf_name)
        delta_th=tf.transformations.euler_from_quaternion(rot)[2]
        print (pose[1], delta_th, active)
        
        if (abs(delta_th)>0.1):pose[1]=0
        
        omni_base.tiny_move( velX=0.0,velY=-pose[1], velT=-delta_th,std_time=0.2, MAX_VEL=0.1) 
    return (abs(pose[1])<0.01 and abs(delta_th)<0.1 )

#------------------------------------------------------
def find_best_grid_point(free_grid, radius=0.1):
    tree = KDTree(free_grid)
    max_neighbors = 0
    best_point = None
    for point in free_grid:
        neighbors = tree.query_ball_point(point, radius)
        if len(neighbors) > max_neighbors:
            max_neighbors = len(neighbors)
            best_point = point
    return best_point
#------------------------------------------------------
def get_keywords_speech(timeout=5):
    try:
        msg = rospy.wait_for_message('/speech_recognition/final_result', String, timeout)
        result = msg.data
        return result
            
    except ROSException:
        rospy.loginfo('timeout')
        return 'timeout'

#------------------------------------------------------
def check_room_px(px_pose,living_room_px_region,kitchen_px_region,bedroom_px_region,dining_room_px_region ):

    
    for i in range(4):
        if i==0:
            px_region=living_room_px_region
            region='living_room'
        if i==1:
            px_region=kitchen_px_region
            region='kitchen'
        if i==2:
            px_region=bedroom_px_region
            region='bedroom'
        if i==3:
            px_region=dining_room_px_region
            region='dining_room'
        #print (region,px_region,px_pose)
        if (px_pose[1]< px_region[1,1]) and (px_pose[1]> px_region[0,1]) and (px_pose[0]> px_region[0,0]) and (px_pose[0]< px_region[1,0]) : 
            print (f'in  {region}')
            return region

#-----------------------------------------------------------------
def save_image(img,name='',dirName=''):
    rospack = rospkg.RosPack()
    file_path = rospack.get_path('images_repos')
    
    num_data = len(glob(path.join(file_path,"src",dirName,"*"))) if dirName else len(glob(path.join(file_path,"src","*")))
    
    num_data = str(num_data+1).zfill(4)

    name = "/" + name if (name and not(name.startswith("/"))) else name
    dirName = "/" + dirName if (dirName and not(dirName.startswith("/"))) else dirName

 
    if name and dirName:
        #print(file_path+"/src"+dirName+name+".jpg")
        cv2.imwrite(file_path+"/src"+dirName+name+num_data+".jpg",img)
    
    elif dirName and not(name):
        #print(file_path+"/src"+dirName+"/"+"image"+".jpg")
        cv2.imwrite(file_path+"/src"+dirName+"/"+"image"+num_data+".jpg",img)

    elif not(dirName) and name:
        #print(file_path+"/src"+name+".jpg")
        cv2.imwrite(file_path+"/src"+name+num_data+".jpg",img)
    
    else:
        #print(file_path+"/src"+"tmp"+".jpg")
        cv2.imwrite(file_path+"/src"+"image"+".jpg",img)
    

#------------------------------------------------------
# para quitar fondo, es necesario rgbd de la camara del robot
def removeBackground(points_msg,distance = 2):
    # Obtengo rgb
    points_data = ros_numpy.numpify(points_msg)
    image_data = points_data['rgb'].view((np.uint8, 4))[..., [2, 1, 0]]   
    image=cv2.cvtColor(image_data, cv2.COLOR_BGR2RGB)
    image = points_data['rgb'].view((np.uint8, 4))[..., [2, 1, 0]]
    rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)    

    # Quito todos los pixeles que esten a una distancia mayor y/o a una distancia menor
    # Para poder obtener una mascara con ceros y unos
    zs_no_nans=np.where(~np.isnan(points_data['z']),points_data['z'],10)
    img_corrected = np.where((zs_no_nans < distance + 0.3),zs_no_nans,0)
    #img_corrected = np.where((img_corrected >1.5),img_corrected,0)
    img_corrected = np.where((img_corrected == 0),img_corrected,1)

    # operacion AND entre la imagen original y la mascara para quitar fondo (background)
    #img_corrected = img_corrected.astype(np.uint8)
    masked_image = cv2.bitwise_and(rgb_image, rgb_image, mask=img_corrected.astype(np.uint8))
    return rgb_image, masked_image


#------------------------------------------------------
def load_rooms_areas_stickler(fileName=''):
    rospack = rospkg.RosPack()
    file_path = rospack.get_path('config_files')
    if fileName =='':
        room_regions=np.load(file_path+'/room_regions_stickler.npy')
    else:
        room_regions=np.load(file_path+'/'+fileName)
    # ORDEN: living_room,kitchen,bedroom,dining_room
    return np.asarray(room_regions[0]),np.asarray(room_regions[1]),np.asarray(room_regions[2]),np.asarray(room_regions[3])


#------------------------------------------------------
def get_robot_person_coords(pose,fileName=''):
    living_room_px_region,kitchen_px_region,bedroom_px_region,dining_room_px_region = load_rooms_areas_stickler(fileName)
    origin_map_img=[round(img_map.shape[0]*0.5) ,
                    round(img_map.shape[1]*0.5)]
    px_pose_human=np.asarray(([origin_map_img[1]+ round(pose[1]/pix_per_m),
                               origin_map_img[0]+ round(pose[0]/pix_per_m)]))
    room_human =check_room_px(np.flip(px_pose_human),
                              living_room_px_region,
                              kitchen_px_region,
                              bedroom_px_region,
                              dining_room_px_region)
    poseRob=get_robot_px()
    px_pose_robot=np.asarray((origin_map_img[1]+poseRob[1],
                              origin_map_img[0]+poseRob[0]))

    room_robot = check_room_px(np.flip(px_pose_robot),
                               living_room_px_region,
                               kitchen_px_region,
                               bedroom_px_region,
                               dining_room_px_region)
    return room_robot,room_human
#------------------------------------------------------
def detect_human_to_pt_st(dist = 6,remove_bkg = True):
    req = Human_detectorRequest()
    req.dist = dist
    req.removeBKG = remove_bkg
    humanpose=human_detect_server(req)
    print ("humanpose",humanpose)
    if np.all(np.array([humanpose.x, humanpose.y, humanpose.z]) == 0):
        #print ("ASARRAY",np.asarray((humanpose.x,humanpose.y,humanpose.z)))
        return False
    else:
        #tf_man.pub_static_tf(np.asarray((humanpose.x,humanpose.x,humanpose.z)),point_name='human', ref='head_rgbd_sensor_link')
        rospy.sleep(0.5)
        point_st= PointStamped()
        point_st.header.stamp = rospy.Time.now()
        point_st.header.frame_id = "head_rgbd_sensor_rgb_frame"#'odom'  # or whatever frame you're using
        point_st.point.x = humanpose.x  # replace with your x coordinate
        point_st.point.y = humanpose.y  # replace with your y coordinate
        point_st.point.z =  0.0 ###FLOOR ### humanpose.z  # replace with your z coordinate
        ##################################
        point_odom = tfBuffer.transform(point_st, "odom", timeout=rospy.Duration(1))

      
        
        
        pt_pub.publish(point_odom)
        return True

       

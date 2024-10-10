#!/usr/bin/env python3

import rospy 
import tf2_ros
import smach
import smach_ros
import rospkg
import actionlib
import tf
import math
from glob import glob
from os import path
import numpy as np
import cv2
from face_recog.msg import *
from face_recog.srv import *
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point, Quaternion
from tf2_geometry_msgs import PointStamped , PoseStamped  
from act_recog.srv import Recognize,RecognizeResponse,RecognizeRequest
from hmm_act_recog.srv import RecognizeOP,RecognizeOPResponse,RecognizeOPRequest
from human_detector.srv import Human_detector  ,Human_detectorRequest 
from cv_bridge import CvBridge, CvBridgeError
from hmm_navigation.msg import NavigateActionGoal , NavigateActionResult

from utils.grasp_utils import GAZE,ARM,WRIST_SENSOR
from utils.misc_utils import talk, TF_MANAGER, RGBD
from utils.nav_utils import OMNIBASE

global listener, broadcaster, tfBuffer, tf_static_broadcaster,recognize_action_docker,recognize_action,human_detect_server,bridge,pt_pub
global head, brazo ,tf_man,wrist,omni_base

rospy.init_node('smach', anonymous=True)

tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)
broadcaster = tf2_ros.TransformBroadcaster()
tf_static_broadcaster = tf2_ros.StaticTransformBroadcaster()

recognize_action_docker = rospy.ServiceProxy('recognize_act_docker', Recognize) 
recognize_action = rospy.ServiceProxy('recognize_act', RecognizeOP) 
human_detect_server = rospy.ServiceProxy('/detect_human' , Human_detector)  ####HUMAN FINDER OPPOSEBASED
recognize_face = rospy.ServiceProxy('recognize_face', RecognizeFace)  
pt_pub = rospy.Publisher('/clicked_point', PointStamped, queue_size=1)  
navclient = actionlib.SimpleActionClient('/move_base/move', MoveBaseAction)

head = GAZE()
brazo = ARM()
tf_man = TF_MANAGER()
bridge = CvBridge()
omni_base=OMNIBASE() 
wrist= WRIST_SENSOR()
rgbd= RGBD()
#-------------------------------------------
def wait_for_push_hand(time=10):

    start_time = rospy.get_time()
    time= 10
    print('timeout will be ',time,'seconds')
    while rospy.get_time() - start_time < time:
        torque = wrist.get_torque()
        if np.abs(torque[1])>1.0:
            print(' Hand Pushed Ready TO start')
            #takeshi_talk_pub.publish(string_to_Voice())
            talk('Ready to start')
            return True
            break

    if (rospy.get_time() - start_time >= time):
        print(time, 'secs have elapsed with no hand push')
        return False

#-------------------------------------------
def human_xyz_to_pt_st(person_xyz):
    
    #tf_man.pub_static_tf(np.asarray((humanpose.x,humanpose.x,humanpose.z)),point_name='human', ref='head_rgbd_sensor_link')
    rospy.sleep(0.5)
    point_st= PointStamped()
    point_st.header.stamp = rospy.Time.now()
    point_st.header.frame_id = "head_rgbd_sensor_rgb_frame"#'odom'  # or whatever frame you're using
    point_st.point.x = person_xyz[0]  # replace with your x coordinate
    point_st.point.y = person_xyz[1]  # replace with your y coordinate
    point_st.point.z = person_xyz[2] ###FLOOR ### humanpose.z  # replace with your z coordinate
    ##################################
    point_odom = tfBuffer.transform(point_st, "odom", timeout=rospy.Duration(1))
    print (point_odom)

        
    pt_pub.publish(point_odom)
    return True

#-------------------------------------------
def move_base_no_map(goal_x,goal_y,goal_yaw,time_out=10):

    #using nav client and toyota navigation go to x,y,yaw
    #To Do: PUMAS NAVIGATION
    pose = PoseStamped()
    pose.header.stamp = rospy.Time(0)
    pose.header.frame_id = "map"
    pose.pose.position = Point(goal_x, goal_y, 0)
    quat = tf.transformations.quaternion_from_euler(0, 0, goal_yaw)
    pose.pose.orientation = Quaternion(*quat)


    # create a MOVE BASE GOAL
    goal = MoveBaseGoal()
    goal.target_pose = pose

    # send message to the action server
    navclient.send_goal(goal)

    # wait for the action server to complete the order
    navclient.wait_for_result(timeout=rospy.Duration(time_out))

    # print result of navigation
    action_state = navclient.get_state()
    return navclient.get_state()

#-------------------------------------------
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
def check_dist_INITIAL(userdata):
    res, userdata.face_img = wait_for_face()
    return True



#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Apr 17 22:54:18 2019

@author: oscar
"""
import sys
print("Python path:", sys.executable)

from geometry_msgs.msg import Quaternion , Point
import numpy as np
import rospy
import message_filters
import cv2
from std_msgs.msg import String ,ColorRGBA
from sensor_msgs.msg import LaserScan , Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped 
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker , MarkerArray
from tf.transformations import *
from tf.msg import tfMessage
from tf.transformations import euler_from_quaternion
from utils_hmm import  viterbi
from utils_hmm import forw_alg
from utils_hmm import backw_alg , save_viterbi_results
from joblib import dump, load
import matplotlib.pyplot as plt
import math
import rospkg
#################
import cv2
import tensorflow
from tensorflow.keras.applications.inception_resnet_v2 import InceptionResNetV2, preprocess_input
from cv_bridge import CvBridge, CvBridgeError

import torch
from timm import create_model
from torchvision.transforms import transforms

bridge = CvBridge()
img_width, img_height = 224, 224  # Adjust for DeiT input
kill_node = False
obs = []

# Load DeiT-Tiny model
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
model = create_model('deit_tiny_patch16_224', pretrained=True)
model.head = torch.nn.Identity()  # Remove the classification head
model.eval().to(device)

# Preprocessing pipeline
preprocess = transforms.Compose([
    transforms.ToPILImage(),
    transforms.Resize((img_width, img_height)),
    transforms.ToTensor(),
    transforms.Normalize(mean=[0.5, 0.5, 0.5], std=[0.5, 0.5, 0.5])  # Match DeiT normalization
])


#img_width,img_height=600,600
#model=InceptionResNetV2(weights='imagenet',include_top=False, input_shape=(img_width, img_height, 3))
#########################################

odom_adjust,odom_adjust_aff=np.zeros(3),np.zeros(3)
first_adjust= np.zeros(3)
xyth_hmms=np.zeros(3)
hmm12real=np.zeros(3)
xyth_odom_prueba=np.zeros(3)
first=True
real_state=[]
last_states=[]
last_states_2=[]
delta_xyth=[]
o_k=[]
o_k2=[]
#transitions= np.load('trans.npy')
buf_vit=150
#clf=load('aff_prop_class.joblib_2')
marker=Marker()   
markerarray=MarkerArray()
first_adjust_2=True
last_states_trans=[0,0]



class HMM (object):
             def __init__(self,A,B,PI):
                 self.A=A
                 self.B=B
                 self.PI=PI  

### LOAD MODEL A QUANTIZERS 
file_path = rospkg.RosPack().get_path('hmm_navigation') + '/scripts/hmm_nav/'
ccvk = np.load(file_path + 'ccvk.npy')
ccvk_v = np.load(file_path + 'ccvk_v.npy')
ccxyth = np.load(file_path + 'ccxyth.npy')
print (f'centroids symb {ccvk.shape}, states {ccxyth.shape}  ')
#########################################################################



#############
A, B, PI=    np.load(file_path +'A.npy') , np.load(file_path +'B.npy') , np.load(file_path +'PI.npy')
Modelo1= HMM(A,B,PI)
A2, B2, PI2= np.load(file_path +'A2.npy') , np.load(file_path +'B2.npy') , np.load(file_path +'PI2.npy')## SAME MATRIX A BUT COULD NOT BE
Modelo2= HMM(A,B2,PI2)
###################################################
print (f'Models B Shape for sanity check Lidar->{B.shape},Vision-> {B2.shape} ')


    

def callback(laser,pose,odom , img_msg):
        global xyth , xyth_odom , xyth_hmms, hmm12real , xyth_odom_prueba , ccxyth , ccvk , A
        global first , last_states_trans
        global odom_adjust,odom_adjust_aff,first_adjust , first_adjust_2

        
        #GET REAL ROBOT POSE 
        text_to_rviz=""
        x_odom=odom.pose.pose.position.x
        y_odom=odom.pose.pose.position.y
        quaternion = (
        odom.pose.pose.orientation.x,
        odom.pose.pose.orientation.y,
        odom.pose.pose.orientation.z,
        odom.pose.pose.orientation.w)
        euler = euler_from_quaternion(quaternion)
        th_odom=euler[2]
        
        
        
        lec=np.asarray(laser.ranges)
        lec[np.isinf(lec)]=13.5
        lec=np.clip(lec,0,5)
        lec_str=str(lec[0])+','
        for i in range (1,len(lec)):
            lec_str=lec_str+str(lec[i])+',' 
            #print (lec_str)
        lec.reshape(len(laser.ranges),1 )

        quaternion = (
        pose.pose.orientation.x,
        pose.pose.orientation.y,
        pose.pose.orientation.z,
        pose.pose.orientation.w)
        euler = euler_from_quaternion(quaternion)
        #roll = euler[0]
        #pitch = euler[1]
        #yaw = euler[2]
        

        #cv2_img = bridge.imgmsg_to_cv2(img_msg, "bgr8")
        #img_resized=cv2.resize(cv2_img,(img_width,img_height))
        #inp_img= np.expand_dims(img_resized ,axis=0)
        #feature_vec = model.predict(inp_img)[0,0,0,:]

        cv2_img = bridge.imgmsg_to_cv2(img_msg, "bgr8")
        img_tensor = preprocess(cv2_img).unsqueeze(0).to(device)

        # Feature extraction
        with torch.no_grad():
            feature_vec = model(img_tensor).cpu().numpy().flatten()
        print(f"Features shape: {feature_vec.shape}")        

        symbol2= np.power(feature_vec-ccvk_v,2).sum(axis=1,keepdims=True).argmin()   ### VISION RESNET
        symbol= np.power(lec.T-ccvk,2).sum(axis=1,keepdims=True).argmin()            ### lIDAR
        
        
        
        
        
        if len(o_k) >=buf_vit:
            o_k.pop(0)
        if len(o_k2) >=buf_vit:
            o_k2.pop(0)
        o_k.append(symbol)
        o_k2.append(symbol2)
        xyth= np.asarray((pose.pose.position.x,pose.pose.position.y,euler[2]))
        xyth_odom=np.asarray((x_odom, y_odom,th_odom))
        #delta_xyth.append(xyth)
        delta_xyth.append(xyth_odom)
        if (len(delta_xyth)>2):
            delta_xyth.pop(0)
            delta= delta_xyth[1]-delta_xyth[0]
            delta_phase=math.atan2(delta[0],delta[1])
            delta_mag=np.linalg.norm(delta[:2])
            print('xyth[-1]-xyth_odom[-1]',xyth[-1]-xyth_odom[-1])
            delta_phase_rotated=delta_phase  - (xyth_hmms[-1]-xyth_odom[-1])
            deltarotated=np.asarray( (delta_mag*math.sin(delta_phase_rotated),delta_mag*math.cos(delta_phase_rotated) , delta[2]   ) )
            xyth_hmms+=deltarotated

            xyth_odom_prueba+=delta 
            


        #QUANTIZING READS (CLASIFIYNG ok2)
       
        xythcuant=np.argmin(np.linalg.norm(xyth-ccxyth,axis=1))
        xyth_odomcuant=np.argmin(np.linalg.norm(xyth-ccxyth,axis=1))

        

        if (len(o_k2)< buf_vit):print ( "FILLING BUFFER HMM2")
        if (len(o_k )< buf_vit):print ( "FILLING BUFFER HMM1")
        
        if (len(o_k)>= buf_vit) and (len(o_k2)>= buf_vit):
        
            
            vit_est= viterbi(o_k,Modelo1,Modelo1.PI)
            vit_est_2= viterbi(o_k2,Modelo2,Modelo2.PI)
                
            
            print (f'Most likely state seq given O{vit_est}')#,vit_est)[-5:])
            print (f'Most likely states given O Modelo DeiT-Tiny ({vit_est_2})') #[-5:])
            print (f'Real last states{real_state}')
            save_viterbi_results(real_state, vit_est, vit_est_2, 'viterbi_results_DeiT-Tiny.txt')



        #xythcuant=np.argmin(np.linalg.norm(xyth-ccxyth,axis=1))
        #print ('Pose',xyth)
        #print (f"ccxyth[ {xythcuant}]={ccxyth[xythcuant]}")
        xyth[2]=0.1*xyth[2]   
        xythcuant=np.argmin(np.linalg.norm(xyth-ccxyth,axis=1))
        real_state.append(xythcuant)
        if len(real_state)==buf_vit:real_state.pop(0)

        print (f"ccxyth[ {xythcuant}]={ccxyth[xythcuant]}")
        #print('lec vk_aff'+str(Vk_aff)+'lec vk'+str(symbol)  +') , ccxyth[ '+str(xythcuant)+']='+str(ccxyth[xythcuant]) + '\n')
        print ('Pose',xyth)
        print('Wheel ' ,xyth_odom)
        print('Dual HMM' ,xyth_hmms)
        print('xyth_odom_prueba',xyth_odom_prueba)


        text_to_rviz+= 'REAL Pose'+ str(xyth)+'\n'+'Wheel odom  '+ str(xyth_odom)+'\n'+'Dual HMMS'+ str(xyth_hmms)+'\n'


        
        




        markerarray=MarkerArray()
        marker= Marker()
        marker.header.frame_id="/map"
        marker.header.stamp = rospy.Time.now()
        marker.id=1

        
        marker.scale.z = 0.2
        marker.pose.position.x = xyth[0]+0.5
        marker.pose.position.y = xyth[1]+0.5  
        marker.pose.position.z = 0
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.type = marker.TEXT_VIEW_FACING
        marker.action = marker.ADD
        marker.text=text_to_rviz
        markerarray.markers.append(marker)
        pub.publish(markerarray)

        
       

        """    with  open('dataset_candidatura_wr/estimadores.txt' , 'a') as out:
                            text_line=""
                            for value in xyth_odom:
                                text_line+=str(value)+','
                            for value in xyth_odom-odom_adjust:
                                text_line+=str(value)+','
                            for value in xyth_odom-odom_adjust_aff:
                                text_line+=str(value)+','
                            for value in xyth:
                                text_line+=str(value)+','        
                            text_line+= str(last_states[-1])+','+str(last_states[-1])+','
                            text_line+='\n'
                            out.write(text_line)
                    """    
    
def listener():
    
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    #clf=load('aff_prop_class.joblib')
    global pub , pub2
    rospy.init_node('listener', anonymous=True)
    #twist = message_filters.Subscriber('cmd_vel',Twist)
    symbol= message_filters.Subscriber('/hsrb/base_scan',LaserScan)
    image= message_filters.Subscriber("/hsrb/head_rgbd_sensor/rgb/image_rect_color",Image)
    pub= rospy.Publisher('aa/Viterbi',MarkerArray,queue_size=1)
    pub2 = rospy.Publisher('/aa/HMM_topo/', MarkerArray, queue_size=1)  
    #pose  = message_filters.Subscriber('/navigation/localization/amcl_pose',PoseWithCovarianceStamped)#TAKESHI REAL
    pose  = message_filters.Subscriber('/hsrb/base_pose',PoseStamped)#TAKESHI GAZEBO
    odom= message_filters.Subscriber("/hsrb/wheel_odom",Odometry)
    #ats= message_filters.ApproximateTimeSynchronizer([symbol,odom,twist],queue_size=5,slop=.1,allow_headerless=True)
    ats= message_filters.ApproximateTimeSynchronizer([symbol,pose,odom, image],queue_size=5,slop=.1,allow_headerless=True)
    ats.registerCallback(callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    
    
    
   
    listener()

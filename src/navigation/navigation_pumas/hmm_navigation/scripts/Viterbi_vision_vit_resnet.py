#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Apr 17 22:54:18 2019

@author: oscar
"""
import sys
import tf
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
first=True
bridge = CvBridge()
img_width, img_height = 224, 224  # Adjust for DeiT input
obs = []
#####################

###################
# Load DeiT-Tiny model
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
model = create_model('deit_tiny_patch16_224', pretrained=True)
model.head = torch.nn.Identity()  # Remove the classification head
model.eval().to(device)
#############################################
def process_viterbi(xyth,vit_est,ccxyth):
    xyth_odom=xyth.copy()
    if len (np.unique(vit_est)>1):
            print ("YOOOOOOOOOOOOOO  \n")
            unique_in_order = np.unique(vit_est, return_index=True)
            unique_in_order = vit_est[np.sort(unique_in_order[1])]
            last_state = ccxyth[int(unique_in_order[-1])]
            second_last_state = ccxyth[int(unique_in_order[-2])]
            print(f'last state  {last_state},{int(unique_in_order[-1])},one to last state{second_last_state},{int(unique_in_order[-2])} ')
            xyth_odom[:2] = (np.array(last_state)[:2] + np.array(second_last_state)[:2]) / 2
            print (f"YOOOOOOOOOOOOOO {xyth_odom}, or { ccxyth[int(np.unique(vit_est)[-1])]    } \n")
            
            return xyth_odom
    print ( 'No lenth. Give centroid ( higher quantization error)')
    #return xyth_odom    
    print ("YOOOOOOOOOOOOOO  NOID  \n \n \n \n\n ")
    return ccxyth[int(np.unique(vit_est)[-1])]    
# Preprocessing pipeline
preprocess = transforms.Compose([
    transforms.ToPILImage(),
    transforms.Resize((img_width, img_height)),
    transforms.ToTensor(),
    transforms.Normalize(mean=[0.5, 0.5, 0.5], std=[0.5, 0.5, 0.5])  # Match DeiT normalization
])


img_width_resnet,img_height_resnet=600,600
model_r=InceptionResNetV2(weights='imagenet',include_top=False, input_shape=(img_width_resnet, img_height_resnet, 3))
#########################################

odom_adjust,odom_adjust_aff=np.zeros(3),np.zeros(3)
first_adjust= np.zeros(3)
xyth_hmms=np.zeros(3)
hmm12real=np.zeros(3)
xyth_odom_lidar=np.zeros(3)
xyth_odom_vit=np.zeros(3)
xyth_odom_res=np.zeros(3)
xyth_odom_wheel=np.zeros(3)
xyth_odom_fused=np.zeros(3)
first=True
real_state=[]
last_states=[]
last_states_2=[]
delta_xyth=[]
o_k=[]
o_k2=[]
o_k3=[]
#transitions= np.load('trans.npy')
buf_vit=150
#clf=load('aff_prop_class.joblib_2')
marker=Marker()   
markerarray=MarkerArray()
first_adjust_2=True
last_states_trans=[0,0]

last_vit_state_1 = None
last_vit_state_2 = None
last_vit_state_3 = None  


class HMM (object):
             def __init__(self,A,B,PI):
                 self.A=A
                 self.B=B
                 self.PI=PI  

### LOAD MODEL A QUANTIZERS 
file_path = rospkg.RosPack().get_path('hmm_navigation') + '/scripts/hmm_nav_lab/'

#file_path = rospkg.RosPack().get_path('hmm_navigation') + '/scripts/hmm_nav/'
ccvk = np.load(file_path + 'ccvk.npy')          #LiDar
ccvk_v = np.load(file_path + 'ccvk_v.npy')      #viT    
ccvk_r = np.load(file_path + 'ccvk_r.npy')      #ResNet    
ccxyth = np.load(file_path + 'ccxyth.npy')      #odom
print (f'centroids symb {ccvk.shape}, states {ccxyth.shape}  ')
print (f'centroids symb viT {ccvk_v.shape}, resnet  {ccvk_r.shape}')
#########################################################################



#############
A, B, PI=    np.load(file_path +'A.npy') , np.load(file_path +'B.npy') , np.load(file_path +'PI.npy')
Modelo1= HMM(A,B,PI)
A2r, B2r, PI2r= np.load(file_path +'A-r.npy') , np.load(file_path +'B-r.npy') , np.load(file_path +'PI-r.npy')## SAME MATRIX A BUT COULD NOT BE
Modelo2r= HMM(A2r,B2r,PI2r)
A2v, B2v, PI2v= np.load(file_path +'A2-v.npy') , np.load(file_path +'B2-v.npy') , np.load(file_path +'PI2-v.npy')## SAME MATRIX A BUT COULD NOT BE
Modelo2v= HMM(A2v,B2v,PI2v)
###################################################
print (f'Models B Shape for sanity check Lidar->{B.shape},Vision Transformer-> {B2v.shape}, Resnet->{B2r.shape} ')


 
def callback(laser,pose,odom , img_msg):
        global xyth , xyth_odom , xyth_odom_wheel, hmm12real , xyth_odom_lidar ,xyth_odom_vit ,xyth_odom_res , ccxyth , ccvk , A
        global first , last_states_trans ,xyth_odom_fused
        global odom_adjust,odom_adjust_aff,first_adjust , first_adjust_2 , last_vit_state_1 ,last_vit_state_2 ,last_vit_state_3

       
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
        
        ########READ LiDar
        
        lec=np.asarray(laser.ranges)
        lec[np.isinf(lec)]=13.5
        lec_str=str(lec[0])+','
        for i in range (1,len(lec)):
            lec_str=lec_str+str(lec[i])+',' 
            #print (lec_str)
        lec.reshape(len(laser.ranges),1 )
        symbol= np.power(lec.T-ccvk,2).sum(axis=1,keepdims=True).argmin()            ### lIDAR

        #########################Read Visual Transformer and Resnet
        cv2_img = bridge.imgmsg_to_cv2(img_msg, "bgr8")
        img_tensor = preprocess(cv2_img).unsqueeze(0).to(device)

        # Feature extraction
        with torch.no_grad():
            feature_vec = model(img_tensor).cpu().numpy().flatten()
        #print(f"Features viT De iT shape: {feature_vec.shape}")        
        symbol2= np.power(feature_vec-ccvk_v,2).sum(axis=1,keepdims=True).argmin()   ### VISION Transformer DeIt
        img_resized=cv2.resize(cv2_img,(img_width_resnet,img_height_resnet))
        inp_img= np.expand_dims(img_resized ,axis=0)
        feature_vec_resnet = model_r.predict(inp_img)[0,0,0,:]
        symbol3= np.power(feature_vec_resnet-ccvk_r,2).sum(axis=1,keepdims=True).argmin()   ### Resnet Quantizer
        #print(f"Features ResNet shape: {feature_vec_resnet.shape}")        
        
        if len(o_k) >=buf_vit:o_k.pop(0)
        if len(o_k2) >=buf_vit:o_k2.pop(0)
        if len(o_k3) >=buf_vit:o_k3.pop(0)
        o_k.append(symbol)
        o_k2.append(symbol2)
        o_k3.append(symbol3)
        ###########################################################  
        pose_tf,quat=  tf_listener.lookupTransform('map','base_footprint',rospy.Time(0))
        print (pose_tf, quat)
        euler = euler_from_quaternion(quat)
        xyth= np.asarray((pose_tf[0],pose_tf[1],euler[2]))
        xyth_odom=np.asarray((x_odom, y_odom,th_odom))
        #delta_xyth.append(xyth.copy())
        delta_xyth.append(xyth_odom.copy())

        if (len(delta_xyth)>2):
            if first :
                xyth_odom_lidar=xyth_odom.copy()
                xyth_odom_vit=  xyth_odom.copy() 
                xyth_odom_res=  xyth_odom.copy() 
                xyth_odom_wheel=xyth_odom.copy() 
                xyth_odom_fused=xyth_odom.copy() 
                first = False
            delta_xyth.pop(0)
            delta= delta_xyth[1]-delta_xyth[0]
            delta_phase=np.arctan2 (delta[1],delta[0])
            delta_mag=np.linalg.norm(delta[:2])


            #delta_xyth[0]+delta_mag*np.cos(delta_phase)
            #delta_xyth[1]+delta_mag*np.sin(delta_phase)
            #delta_xyth[2]+delta[2]
            deltarotated_w=np.asarray( (delta_mag*np.cos(delta_phase),delta_mag*np.sin(delta_phase), delta[2]))
            deltarotated=np.asarray( (delta_mag*np.cos(delta_phase-xyth_odom[2]+xyth[2]),delta_mag*np.sin(delta_phase-xyth_odom[2]+xyth[2]), delta[2]))                                            

            #pose_2,print(f' x {pose_1[0]+delta_mag*np.cos(delta_phase-wheel_odom_1[2]+pose_1[2])} ,y {pose_1[1]+delta_mag*np.sin(delta_phase-wheel_odom_1[2]+pose_1[2])}, th {pose_1[2]+delta[2]}')
            #delta_phase_rotated=delta_phase  + (-xyth_odom[-1])

            #print (f'\n \ndelta_phase {delta_phase}, delta mag{delta_mag}, delta {delta}, delta_xyth{delta_xyth}  , robot orientation{xyth_odom[-1]} \n \n')
            
            #deltarotated=np.asarray( (delta_mag*math.sin(delta_phase_rotated),delta_mag*math.cos(delta_phase_rotated) , delta_phase_rotated   ) )
            #deltarotated= delta_phase + 
            
            xyth_odom_lidar+=deltarotated
            xyth_odom_vit+=deltarotated
            xyth_odom_res+=deltarotated
            xyth_odom_fused+=deltarotated
            xyth_odom_wheel+=deltarotated_w
        #QUANTIZING READS (CLASIFIYNG ok2)
       
                
        
        adjusted_xyth=xyth.copy()
        adjusted_xyth[2]=0.1*adjusted_xyth[2]   
        xythcuant=np.argmin(np.linalg.norm(adjusted_xyth[:2]-ccxyth[:,:2],axis=1))
        print(f"Odom{adjusted_xyth},ccxyth[{xythcuant}]=ccxyth{ccxyth[xythcuant]}",)
        real_state.append(xythcuant)
        if len(real_state)>buf_vit:real_state.pop(0)


        #xyth_odomcuant=np.argmin(np.linalg.norm(xyth-ccxyth,axis=1))

        
        if (len(o_k3)< buf_vit):print ( "FILLING BUFFER HMM3 Resnet")
        if (len(o_k2)< buf_vit):print ( "FILLING BUFFER HMM2 DeIt")
        if (len(o_k )< buf_vit):print ( "FILLING BUFFER HMM1 LiDar ")
        
        if (len(o_k)>= buf_vit) and (len(o_k2)>= buf_vit):
        
            
            vit_est= viterbi(o_k,Modelo1,Modelo1.PI)
            vit_est_2= viterbi(o_k2,Modelo2v,Modelo2v.PI)
            vit_est_3= viterbi(o_k3,Modelo2r,Modelo2r.PI)
              
            alpha_r=forw_alg(o_k3,Modelo2r)
            alpha_l=forw_alg(o_k,Modelo1)
            alpha_v=forw_alg(o_k2,Modelo2v)
            

            if vit_est[-1] == real_state[-1]:
                if last_vit_state_1 != vit_est[-1]:

                    xyth_odom_lidar = process_viterbi(xyth, vit_est[1:], ccxyth)
                    xyth_odom_fused = xyth
                    last_vit_state_1 = vit_est[-1]  # Update the tracking variable

            if vit_est_2[-1] == real_state[-1]:
                if last_vit_state_2 != vit_est_2[-1]:
                    xyth_odom_vit = process_viterbi(xyth, vit_est_2[1:], ccxyth)
                    xyth_odom_fused = xyth
                    last_vit_state_2 = vit_est_2[-1]

            if vit_est_3[-1] == real_state[-1]:
                if last_vit_state_3 != vit_est_3[-1]:
                    xyth_odom_res = process_viterbi(xyth, vit_est_3[1:], ccxyth)
                    xyth_odom_fused = xyth
                    last_vit_state_3 = vit_est_3[-1]




            print (f'Most likely state seq given O                {vit_est   [-10:]} ,  likelihood{alpha_l[:,-1].sum()}')#,vit_est)[-5:])
            print (f'Most likely states given O Modelo DeiT-Tiny ({vit_est_2 [-10:]}, likelihood{alpha_v[:,-1].sum()})') #[-5:])
            print (f'Most likely states given O Modelo ResNet (   {vit_est_3 [-10:]}, likelihood{alpha_r[:,-1].sum()})') #[-5:])
            print (f'Real last states                             {real_state[-10:]} ')
            print (f'xyth                                         {xyth} ')

            save_viterbi_results(xyth,real_state, vit_est, vit_est_2,vit_est_3,'viterbi150_results_viT_resnet.txt')
            
            
            with open('viterbi150_results_likelihoods_real_wheel_lid_vit_res.txt', 'a') as f:
                real_str = ','.join(map(str, xyth))
                odom_str    = ','.join(map(str, xyth_odom))
                likel_str= str( alpha_l[:,-1].sum())
                likev_str= str( alpha_v[:,-1].sum())
                liker_str= str( alpha_r[:,-1].sum())
                line = f"{real_str},{odom_str},{likel_str},{likev_str},{liker_str}\n"
                f.write(line)            
        
        
        print('Wheel ' ,xyth_odom)
        print ('Pose_tf',xyth)
        print('xyth_odom_lidar',xyth_odom_lidar)
        print('xyth_odom_vit'  ,xyth_odom_vit)
        print('xyth_odom_res'  ,xyth_odom_res)
        print('xyth_odom_fused',xyth_odom_fused)
        with open('viterbi150_results_odoms_wheel_real_lid_vit_res.txt', 'a') as f:
            odom_str = ','.join(map(str, xyth_odom))
            real_str = ','.join(map(str, xyth))
            lidar_str= ','.join(map(str, xyth_odom_lidar))
            vit_str  = ','.join(map(str,   xyth_odom_vit))
            res_str  = ','.join(map(str,   xyth_odom_res))
            fused_str= ','.join(map(str,   xyth_odom_fused))

            line = f"{odom_str},{real_str},{lidar_str},{vit_str},{res_str},{fused_str}\n"
            f.write(line)            
        #save_viterbi_results(xyth_odom,xyth, xyth_odom_lidar,xyth_odom_vit ,xyth_odom_res,'viterbi150_results_odoms_wheel_real_lid_vit_res.txt')










        text_to_rviz+= 'REAL Pose'+ str(xyth)+'\n'+'Wheel odom  '+ str(xyth_odom)+'\n'+'Dual HMMS'+ str(xyth_hmms)+'\n'


        
        




        markerarray=MarkerArray()
        marker= Marker()
        marker.header.frame_id="/odom"
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
    global pub , pub2,tf_listener
    rospy.init_node('listener', anonymous=True)
    #twist = message_filters.Subscriber('cmd_vel',Twist)
    symbol= message_filters.Subscriber('/hsrb/base_scan',LaserScan)
    image= message_filters.Subscriber("/hsrb/head_rgbd_sensor/rgb/image_rect_color",Image)
    pub= rospy.Publisher('Viterbi_odom',MarkerArray,queue_size=1)
    tf_listener = tf.TransformListener()
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

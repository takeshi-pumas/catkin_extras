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
import os
import torch
from timm import create_model
from torchvision.transforms import transforms

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
xyth_odom_prueba=np.zeros(3)
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



class HMM (object):
             def __init__(self,A,B,PI):
                 self.A=A
                 self.B=B
                 self.PI=PI  


def callback(laser,pose,odom , img_msg):
        global xyth , xyth_odom , xyth_hmms, hmm12real , xyth_odom_prueba , ccxyth , ccvk , A
        global first , last_states_trans
        global odom_adjust,odom_adjust_aff,first_adjust , first_adjust_2

        
        #GET REAL ROBOT POSE 
        
        xyth = np.asarray((odom.pose.pose.position.x,
                       odom.pose.pose.position.y,
                       euler_from_quaternion([
                           odom.pose.pose.orientation.x,
                           odom.pose.pose.orientation.y,
                           odom.pose.pose.orientation.z,
                           odom.pose.pose.orientation.w])[2]))
        rospy.loginfo(f"Odometry: {xyth}")
        ########READ LiDar
        
        lec=np.asarray(laser.ranges)
        lec[np.isinf(lec)]=13.5
        lec_str = ','.join(map(str, lec))
        print(f'lidar reading length {len(lec)}')

        #########################Read Visual Transformer


        cv2_img = bridge.imgmsg_to_cv2(img_msg, "bgr8")
        img_tensor = preprocess(cv2_img).unsqueeze(0).to(device)

        # Feature extraction
        with torch.no_grad():
            feature_vec = model(img_tensor).cpu().numpy().flatten()
        print(f"Features viT De iT shape: {feature_vec.shape}")   
        # Combine features and save
        feature_str = ','.join(map(str, feature_vec))     

        img_resized=cv2.resize(cv2_img,(img_width_resnet,img_height_resnet))
        inp_img= np.expand_dims(img_resized ,axis=0)
        feature_vec_resnet = model_r.predict(inp_img)[0,0,0,:]
        #symbol3= np.power(feature_vec_resnet-ccvk_r,2).sum(axis=1,keepdims=True).argmin()   ### Resnet Quantizer
        print(f"Features ResNet shape: {feature_vec_resnet.shape}")        
        feature_res_str = ','.join(map(str, feature_vec_resnet))     
        texto = f"{feature_res_str},{feature_str},{lec_str},{xyth[0]},{xyth[1]},{xyth[2]}"

        #with open('~/Documents/resnet_deit_lidar_odom.txt', 'a') as out:
        file_path = os.path.expanduser('~/Documents/resnet_deit_lidar_odom.txt')
        # Open the file in append mode
        with open(file_path, 'a') as out:    
            out.write(texto + '\n')

       

       
    
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
    #odom= message_filters.Subscriber("/hsrb/wheel_odom",Odometry)
    odom= message_filters.Subscriber("/hsrb/odom",Odometry)
    #ats= message_filters.ApproximateTimeSynchronizer([symbol,odom,twist],queue_size=5,slop=.1,allow_headerless=True)
    ats= message_filters.ApproximateTimeSynchronizer([symbol,pose,odom, image],queue_size=5,slop=.1,allow_headerless=True)
    ats.registerCallback(callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    
    
    
   
    listener()

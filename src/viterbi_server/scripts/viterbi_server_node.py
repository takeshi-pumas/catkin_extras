#!/usr/bin/env python3

import rospy
import tf2_ros
from viterbi_server.msg import Obs_symbols ,States_estimated
from viterbi_server.srv import Viterbify ,ViterbifyRequest , ViterbifyResponse
from rospy.numpy_msg import numpy_msg
from utils_hmm import  *
import rospkg
import numpy as np



def callback(req):
    
    print ('length of observations sequence received ', len(req.data.data))    
    

    states= States_estimated()
    states.data.append(req.data.data[0])
    
    vit_est= viterbi(np.ones((1,7)),Modelo1,Modelo1.PI)
    print ('vit_est',vit_est)

    return ViterbifyResponse(states)
    
    
    
    
    


def classify_server():
    global listener,rgbd, bridge, rospack , Modelo1
    rospy.init_node('viterbi_server_node')
    #rgbd= RGBD()
    #bridge = CvBridge()
    #listener = tf.TransformListener()
    #broadcaster= tf.TransformBroadcaster()
    tf_static_broadcaster= tf2_ros.StaticTransformBroadcaster()
    rospack = rospkg.RosPack()
    file_path = rospack.get_path('viterbi_server')
    file_path+='/scripts/matrices/'
    
    rospy.loginfo("Viterbi service available")                    # initialize a ROS node
    A, B, PI= np.load(file_path+'A.npy') , np.load(file_path+'B.npy') , np.load(file_path+'PI.npy')
    #A=np.zeros((3,3))
    #B=np.zeros((3,6))
    #PI=np.zeros((1,3))
    Modelo1= HMM(A,B,PI)
    s = rospy.Service('viterbify', Viterbify, callback) 
    #print("Classification service available")
   

    rospy.spin()

if __name__ == "__main__":
    classify_server()

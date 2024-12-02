# -*- coding: utf-8 -*-
"""
Created on Fri Sep 27 16:25:07 2019

@author: oscar
"""
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist , PointStamped , Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker , MarkerArray
import numpy as np
import pandas as pd

def save_viterbi_results(xyth,real_state, vit_est, vit_est_2,vit_est_3, filename='viterbi_results.txt'):
    """
    Save Viterbi results to a text file in the format:
    real_state, vit_est, vit_est_2

    Args:
        real_state (list or array): The real states.
        vit_est (list or array): The most likely state sequence for the first model.
        vit_est_2 (list or array): The most likely state sequence for the second model.
        filename (str): The name of the text file to write to.
    """
    try:
        with open(filename, 'a') as f:
            # Convert all inputs to strings and join with commas for a single line
            odom_str = ','.join(map(str, xyth))
            real_state_str = ','.join(map(str, real_state))
            vit_est_str = ','.join(map(str, vit_est))
            vit_est_2_str = ','.join(map(str, vit_est_2))
            vit_est_3_str = ','.join(map(str, vit_est_3))
            
            # Concatenate in the desired order: real_state, vit_est, vit_est_2
            line = f"{odom_str},{real_state_str},{vit_est_str},{vit_est_2_str},{vit_est_3_str}\n"
            
            # Write to file
            f.write(line)
        print(f"Results saved to {filename}")
    except Exception as e:
        print(f"Error while saving results: {e}")


def list_2_markers_array(path, ccxyth):
    xythpath=[]
    marker=Marker() 
    markerarray=MarkerArray()    
    for n in path:
        print(n)
        marker.header.frame_id="/map"
        marker.id=n
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = ccxyth[n][0]
        marker.pose.position.y = ccxyth[n][1]  
        marker.pose.position.z = 0
        
        markerarray.markers.append(marker)
        marker=Marker() 
        
        
        
    return np.array(markerarray)

def viterbi(obs,Modelo1,PI):
    A, B= Modelo1.A , Modelo1.B    
    delta=np.zeros((len(obs)+1,len(Modelo1.A)))
    phi=np.zeros((len(obs)+1,len(A)))+666
    path =np.zeros(len(obs)+1)
    T=len(obs)
    Modelo1.PI = PI
    delta[0,:]= Modelo1.PI * Modelo1.B[:,obs[0]]
    phi[0,:]=666
    for t in range(len(obs)):
        for j in range(delta.shape[1]):

            delta [t+1,j]=np.max(delta[t] * A[:,j]) * B[j,obs[t]]
            phi[t+1,j]= np.argmax(delta[t] * A[:,j])
    path[T]=int(np.argmax(delta[T,:]))
    for i in np.arange(T-1,0,-1):
        #print (i,phi[i+1,int(path[i+1])])
        path[i]=phi[i+1,int(path[i+1])]
    return(path)

class HMM (object):
             def __init__(self,A,B,PI):
                 self.A=A
                 self.B=B
                 self.PI=PI   
import numpy as np

def forw_alg(o_k, Modelo, epsilon=1e-12):
    """
    Forward algorithm with log-space trick and handling of zero probabilities.
    
    Parameters:
    - o_k: Sequence of observations (indices).
    - Modelo: Model object containing:
        - A: Transition probabilities (NxN matrix).
        - B: Emission probabilities (NxM matrix, where M is the number of observations).
        - PI: Initial state probabilities (length N).
    - epsilon: Small value to avoid log(0).
    
    Returns:
    - log_alpha: Log of the alpha values (NxK matrix).
    """
    PI = Modelo.PI
    A = Modelo.A
    B = Modelo.B
    K = len(o_k)  # Length of observation sequence
    N = len(A)    # Number of states

    # Add epsilon to avoid log(0)
    A = np.clip(A, epsilon, 1)
    B = np.clip(B, epsilon, 1)
    PI = np.clip(PI, epsilon, 1)

    # Initialize alpha matrix in log-space
    log_alpha = np.zeros((N, K))

    # Initialization (log-space)
    log_alpha[:, 0] = np.log(PI) + np.log(B[:, o_k[0]])

    # Recursion (log-space)
    for k in range(1, K):
        for j in range(N):
            # Compute log-sum-exp for stability
            log_alpha[j, k] = np.log(B[j, o_k[k]]) + np.logaddexp.reduce(log_alpha[:, k-1] + np.log(A[:, j]))

    return log_alpha

def  backw_alg(o_k,Modelo):
    #MATRIX NOTATION

    PI=Modelo.PI
    K= len(o_k)   #Secuencia Observaciones
    N= len(Modelo.A)  #número de estados
    beta=np.zeros((N,K))
    beta[:,-1]=1
    for t in range(K-2,-1,-1):
        beta_t1=beta[:,t+1]
        beta_t1
        a= Modelo.A[:,:]
        b= Modelo.B[:,o_k[t]]
        beta[:,t]= b*np.dot(a,beta_t1)
    return beta
    
    
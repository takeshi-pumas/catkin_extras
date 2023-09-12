#!/usr/bin/env python3
import rospy
import tf2_ros
from viterbi_server.msg import Obs_symbols ,States_estimated
from viterbi_server.srv import Viterbify ,ViterbifyRequest , ViterbifyResponse
from rospy.numpy_msg import numpy_msg
import rospkg
import numpy as np

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

#########################################################################
#########################################################################
class HMM (object):
             def __init__(self,A,B,PI):
                 self.A=A
                 self.B=B
                 self.PI=PI   
def forw_alg(o_k,Modelo):
    #MATRIX NOTATION
    PI=Modelo.PI
    K= len(o_k)   #Secuencia Observaciones
    N= len(Modelo.A)  #número de estados
    alpha=np.zeros((N,K))
    c_k= np.zeros(K)
    alpha[:,0]= PI
    c_k[0]=1
    for k in range(1,K):
        alpha_k= alpha[:,k-1]
        a= Modelo.A[:,:]
        b= Modelo.B[:,o_k[k]]
        alpha[:,k]=(b*np.dot(alpha_k,a))#* c_k[k-1]
        #c_k[k]=1/alpha[:,k].sum()
    return alpha #,c_k
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
    
#########################################################################
#########################################################################   

def callback(req):
    
    print ('length of observations sequence received ', len(req.data.data))    
    

    states= States_estimated()
    
    
    o_k=[]
    for lec in req.data.data:o_k.append(lec)
    
    


    vit_est= viterbi(o_k,Modelo,Modelo.PI)

    for s in vit_est:states.data.append((int)(s))
    print ('vit_est',states)
    
    
    return ViterbifyResponse(states)
    
    
    
    
    


def classify_server():
    global listener,rgbd, bridge, rospack , Modelo
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
    Modelo= HMM(A,B,PI)
    s = rospy.Service('viterbify', Viterbify, callback) 

    #print("Classification service available")
   

    rospy.spin()

if __name__ == "__main__":
    classify_server()

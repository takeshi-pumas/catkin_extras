#!/usr/bin/env python3
from cv_bridge import CvBridge
from sensor_msgs.msg import Image ,PointCloud2
from geometry_msgs.msg import TransformStamped
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import rospy
import ros_numpy
import tf
import tf2_ros as tf2
import numpy as np
import cv2
import sys
from glob import glob
from os import path
#from utils.misc_utils import *
from smach_utils2 import *
try:
	usr_url=path.expanduser( '~' )
	# OJO a este path, cambiarlo desde donde esté la carpeta openpose
    #print(usr_url+'/openpose/build/python')
	sys.path.append(usr_url+'/openpose/build/python');
	from openpose import pyopenpose as op
except ImportError as e:
	print('Error: OpenPose library could not be found. Did you enable `BUILD_PYTHON` in CMake and have this Python script in the right folder?')
	raise e

#global tf_man
#tf_man = TF_MANAGER()

#========================================
class RGB():
    def __init__(self):
        self._img_sub = rospy.Subscriber(
            "/hsrb/head_rgbd_sensor/rgb/image_rect_color",     #FOR DEBUG USB CAM"/usb_cam/image_raw"
            #"/usb_cam/image_raw",                               #"/hsrb/head_rgbd_sensor/rgb/image_rect_color"
            Image, self._img_cb)
        self._image_data = None
        
    def _img_cb(self, msg):
        global bridge
        self._image_data = bridge.imgmsg_to_cv2(msg)
        return
    
    def get_image(self):
        return self._image_data


#---------------------------------------------------
def loadModels(classes):
    # Para cargar los modelos en el codigo de HMM con Vit
    route="src/act_recog/scripts/models/"
    modelsA=[]
    modelsB=[]
    modelsPI=[]

    for cl in classes:
        modelsA.append(np.load(glob(path.join(route,'modelA_'+cl+"*"))[0]))
        modelsB.append(np.load(glob(path.join(route,'modelB_'+cl+"*"))[0]))
        modelsPI.append(np.load(glob(path.join(route,'modelPI_'+cl+"*"))[0]))

    return modelsA,modelsB,modelsPI
#---------------------------------------------------
def create_vk(data,cb,quitaJ=False,centralized=False):
    # Se crean listas vacias para vk
    if quitaJ:
        data=reduce25_to_15(data)
        
    if centralized:
        data=centralizaMatriz(data)
    #data=flat_list(data)
    tmp=data[:,:].ravel(order='F')

    # se obtienen las distancias euclidianas comparando con todos los vectores del codebook
    # retorna la menor de estas distancias

    return np.argmin([np.linalg.norm(tmp-c) for c in cb])

#------------------------------------------------------------------------------
def reduce25_to_15(data):
    # assuming shape of 25,2
    return np.vstack((data[:10,:],data[12,:],data[15:19,:]))

#-------------------------------------------------------------
def centralizaSecuencia(secuencia,codebook=False):
    tmp=np.zeros(secuencia.shape)
    if secuencia.ndim==2:
        for i in range(tmp.shape[0]):
            x=secuencia[i,1]
            y=secuencia[i,int(secuencia[i,:].shape[0]/2)+1]
            tmp[i,:int(secuencia[i,:].shape[0]/2)]=x-secuencia[i,:int(secuencia[i,:].shape[0]/2)]
            tmp[i,int(secuencia[i,:].shape[0]/2):]=y-secuencia[i,int(secuencia[i,:].shape[0]/2):]

    else:
        for i in range(tmp.shape[0]):
            tmp[i,:]=centralizaMatriz(secuencia[i,:])
    return tmp

#-------------------------------------------------------------

def centralizaMatriz(data):
    
    nuevoSK=np.zeros(data.shape)
    if data[1,0]!=0 and data[1,1]!=0:
        coordCentrada=data[1,:]
    else:
        #print("No se encontró esqueleto en joint 1, se utiliza el joint 0")
        coordCentrada=data[0,:]

    for i in range(data.shape[0]):
        if data[i,0]!=0 and data[i,1]!=0:
            nuevoSK[i,:]=coordCentrada[:]-data[i,:]
    return nuevoSK
#---------------------------------------------------
def inf_Secuence(data,modelsA,modelsB,modelsPI):
    
    probas_nuevas = np.zeros((len(modelsA)),dtype=np.float64)
    for i in range(len(modelsA)):
        probas_nuevas[i]=forward(data,modelsA[i],modelsB[i],modelsPI[i])
    return probas_nuevas

#---------------------------------------------------
def forward(V, a, b, initial_distribution):
    probas = np.zeros((V.shape[0], a.shape[0]),dtype=np.float64)
    alpha = np.zeros((a.shape[0]),dtype=np.float64)
    probas[0, :] = initial_distribution * b[:, V[0]]

    for t in range(1, V.shape[0]):
        for j in range(a.shape[0]):
            # Matrix Computation Steps
            #                  ((1x2) . (1x2))      *     (1)
            #                        (1)            *     (1)
            probas[t, j] = probas[t - 1].dot(a[:, j]) * b[j, V[t]]     
    alpha = sum(probas[-1,:])

    return alpha

 #---------------------------------------------------
def init_openPose(n_people=-1,net_res="-1x208",model="BODY_25"):
    try:
        usr_url=path.expanduser( '~/' )
        params = dict()
		# OJO a este path, cambiarlo desde donde esté la carpeta openpose
        #print(usr_url+"openpose/models/")
        params["model_folder"] = usr_url+"openpose/models/"
        params["model_pose"] = model
        params["net_resolution"]= net_res
        # -1 -> toda persona detectable
        params["number_people_max"]= n_people

        opWrapper = op.WrapperPython()
        opWrapper.configure(params)
        opWrapper.start()
        datum = op.Datum()

    except Exception as e:
        print("exception:",e)
        sys.exit(-1)
    return opWrapper,datum

#---------------------------------------------------    
def get_coordinates():
    data=rospy.wait_for_message("/hsrb/head_rgbd_sensor/depth_registered/rectified_points",
            PointCloud2)
    print("datos recibidos")
    # NO SE CORRIGE ( -> correct_points() ), SE OBSERVO QUE NO FUE NECESARIO
    np_data=ros_numpy.numpify(data)
    frame=np_data['rgb'].view((np.uint8, 4))[..., [2, 1, 0]]
    frame=cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    return frame,np_data

#---------------------------------------------------        
def detect_pointing_arm(lastSK,cld_points):
    area=10
    # CodoD -> 3, CodoI -> 6 
    codoD = [cld_points['x'][round(lastSK[3,1]), round(lastSK[3,0])],
            cld_points['y'][round(lastSK[3,1]), round(lastSK[3,0])],
            cld_points['z'][round(lastSK[3,1]), round(lastSK[3,0])]]
    codoD[2]=np.nanmean(np.array(cld_points['z'][round(lastSK[3,1])-area:round(lastSK[3,1])+area+1, 
                                                round(lastSK[3,0])-area:round(lastSK[3,0])+area+1]))
   
    
    # ManoD -> 4, ManoI -> 7
    manoD = [cld_points['x'][round(lastSK[4,1]), round(lastSK[4,0])],
            cld_points['y'][round(lastSK[4,1]), round(lastSK[4,0])],
            cld_points['z'][round(lastSK[4,1]), round(lastSK[4,0])]]
    manoD[2]=np.nanmean(np.array(cld_points['z'][round(lastSK[4,1])-area:round(lastSK[4,1])+area+1, 
                                                round(lastSK[4,0])-area:round(lastSK[4,0])+area+1]))
    
  
    codoI = [cld_points['x'][round(lastSK[6,1]), round(lastSK[6,0])],
            cld_points['y'][round(lastSK[6,1]), round(lastSK[6,0])],
            cld_points['z'][round(lastSK[6,1]), round(lastSK[6,0])]]
    codoI[2]=np.nanmean(np.array(cld_points['z'][round(lastSK[6,1])-area:round(lastSK[6,1])+area+1, 
                                                round(lastSK[6,0])-area:round(lastSK[6,0])+area+1]))
    
   
    manoI = [cld_points['x'][round(lastSK[7,1]), round(lastSK[7,0])],
            cld_points['y'][round(lastSK[7,1]), round(lastSK[7,0])],
            cld_points['z'][round(lastSK[7,1]), round(lastSK[7,0])]]
    manoI[2]=np.nanmean(np.array(cld_points['z'][round(lastSK[7,1])-area:round(lastSK[7,1])+area+1, 
                                                round(lastSK[7,0])-area:round(lastSK[7,0])+area+1]))
   
    
    tf_man.pub_tf(pos=codoD,point_name='codoD_t',ref='head_rgbd_sensor_link')
    tf_man.pub_tf(pos=codoI,point_name='codoI_t',ref='head_rgbd_sensor_link')
    tf_man.pub_tf(pos=manoD,point_name='manoD_t',ref='head_rgbd_sensor_link')
    tf_man.pub_tf(pos=manoI,point_name='manoI_t',ref='head_rgbd_sensor_link')

    # resta entre [0,-1,0] y vectores de codo a mano 
    
    v1=[-(manoD[0]-codoD[0]),-1-(manoD[1]-codoD[1]),-(manoD[2]-codoD[2])]
    v2=[-(manoI[0]-codoI[0]),-1-(manoI[1]-codoI[1]),-(manoI[2]-codoI[2])]
    
    if np.linalg.norm(v1)>np.linalg.norm(v2):
        print("Mano izquierda levantada")
        return manoI,codoI
    else:
        print("Mano derecha levantada")
        return manoD,codoD

#---------------------------------------------------           
def pub_points(cld_points_corrected,lastSK,skPub=1):

    if skPub==1:

        mano,codo=detect_pointing_arm(lastSK,cld_points_corrected)

        print(codo)
        print("algo")
        print(mano)
        tf_man.pub_static_tf(pos=codo,point_name='CODO',ref='head_rgbd_sensor_link')
        tf_man.pub_static_tf(pos=mano,point_name='MANO',ref='head_rgbd_sensor_link')
        #print("cambiando referencia")
        tf_man.change_ref_frame_tf(point_name='CODO',new_frame='map')
        tf_man.change_ref_frame_tf(point_name='MANO',new_frame='map')
        #print("referencia cambiada?")

    else:
        cabeza = [cld_points_corrected['x'][round(lastSK[0,1]), round(lastSK[0,0])],
                cld_points_corrected['y'][round(lastSK[0,1]), round(lastSK[0,0])],
                cld_points_corrected['z'][round(lastSK[0,1]), round(lastSK[0,0])]]
        print(cabeza)
        tf_man.pub_static_tf(pos=cabeza,point_name='cabeza',ref='head_rgbd_sensor_link')
        tf_man.change_ref_frame_tf(point_name='cabeza',new_frame='map')

#---------------------------------------------------
"""
def get_extrapolation(mano,codo,z=0):

    vectD=[mano[0]-codo[0],mano[1]-codo[1],mano[2]-codo[2]]
    alfa=z-mano[2]/vectD[2]
    y=mano[1]+alfa*vectD[1]
    x=mano[0]+alfa*vectD[0]
    
    return [x,y,z]

"""
#---------------------------------------------------

bridge = CvBridge()

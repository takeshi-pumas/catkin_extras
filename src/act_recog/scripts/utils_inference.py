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
from rospkg import RosPack

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

global rospack
rospack = RosPack()
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

class RGBD():
    def __init__(self):
        self._br = tf.TransformBroadcaster()
        self._cloud_sub = rospy.Subscriber(
            "/hsrb/head_rgbd_sensor/depth_registered/rectified_points",
            PointCloud2, self._cloud_cb)
        self._points_data = None
        self._image_data = None
        self._h_image = None
        self._region = None
        self._h_min = 0
        self._h_max = 0
        self._xyz = [0, 0, 0]
        self._frame_name = None

    def _cloud_cb(self, msg):
        self._points_data = ros_numpy.numpify(msg)
        self._image_data = \
        self._points_data['rgb'].view((np.uint8, 4))[..., [2, 1, 0]]
        hsv_image = cv2.cvtColor(self._image_data, cv2.COLOR_RGB2HSV_FULL)
        self._h_image = hsv_image[..., 0]
        self._region = \
        (self._h_image > self._h_min) & (self._h_image < self._h_max)
        if not np.any(self._region):
            return
            
        (y_idx, x_idx) = np.where(self._region)
        x = np.average(self._points_data['x'][y_idx, x_idx])
        y = np.average(self._points_data['y'][y_idx, x_idx])
        z = np.average(self._points_data['z'][y_idx, x_idx])
        self._xyz = [y, x, z]
        if self._frame_name is None:
            return

        self._br.sendTransform(
        (x, y, z), tf.transformations.quaternion_from_euler(0, 0, 0),
        rospy.Time(msg.header.stamp.secs, msg.header.stamp.nsecs),
        self._frame_name,
        msg.header.frame_id)

    def get_image(self):
        return self._image_data

    def get_points(self):
        return self._points_data

    def get_h_image(self):
        return self._h_image

    def get_region(self):
        return self._region

    def get_xyz(self):
        return self._xyz

    def set_h(self, h_min, h_max):
        self._h_min = h_min
        self._h_max = h_max

    def set_coordinate_name(self, name):
        self._frame_name = name

#---------------------------------------------------
        
def loadModels(classes):
    
    # get the file path for rospy_tutorials
    # Para cargar los modelos en el codigo de HMM con Vit
    route=path.join(rospack.get_path("act_recog"))+"/scripts/models/"
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
def init_openPose(n_people=-1,net_res="-1x208",model="BODY_25",heatmap=False):
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
        if heatmap:
            params["heatmaps_add_parts"] = True
            params["heatmaps_add_bkg"] = True
            params["heatmaps_add_PAFs"] = True
            params["heatmaps_scale"] = 2
        
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

    #data=rospy.wait_for_message("/camera/depth/image_raw",PointCloud2)
    
    data=rospy.wait_for_message("/hsrb/head_rgbd_sensor/depth_registered/rectified_points",PointCloud2)
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
   
    if ~np.isnan(manoD).any() or ~np.isnan(codoD).any() or ~np.isnan(manoI).any() or ~np.isnan(codoI).any():
        tf_man.pub_tf(pos=codoD,point_name='codoD_t',ref='head_rgbd_sensor_link')
        tf_man.pub_tf(pos=codoI,point_name='codoI_t',ref='head_rgbd_sensor_link')
        tf_man.pub_tf(pos=manoD,point_name='manoD_t',ref='head_rgbd_sensor_link')
        tf_man.pub_tf(pos=manoI,point_name='manoI_t',ref='head_rgbd_sensor_link')

        # resta entre [0,-1,0] y vectores de codo a mano 
        
        v1=[-(manoD[0]-codoD[0]),-1-(manoD[1]-codoD[1]),-(manoD[2]-codoD[2])]
        v2=[-(manoI[0]-codoI[0]),-1-(manoI[1]-codoI[1]),-(manoI[2]-codoI[2])]
      

        if np.linalg.norm(v1)>np.linalg.norm(v2):
            print("Mano izquierda levantada")
            return manoI,codoI,0
        else:
            print("Mano derecha levantada")
            return manoD,codoD,1
    else:
        return -1,-1,-1

#---------------------------------------------------

def get_extrapolation(mano,codo,z=0):

    vectD=[mano[0]-codo[0],mano[1]-codo[1],mano[2]-codo[2]]
    alfa=z-mano[2]/vectD[2]
    y=mano[1]+alfa*vectD[1]
    x=mano[0]+alfa*vectD[0]
    
    return [x,y,z]


#---------------------------------------------------

bridge = CvBridge()

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
from os import listdir,path

try:
	usr_url=path.expanduser( '~' )
	# OJO a este path, cambiarlo desde donde esté la carpeta openpose
	sys.path.append(usr_url+'/Documentos/openpose/build/python');
	from openpose import pyopenpose as op
except ImportError as e:
	print('Error: OpenPose library could not be found. Did you enable `BUILD_PYTHON` in CMake and have this Python script in the right folder?')
	raise e


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


#--------------------------------------------------------
class TF_MANAGER():
    def __init__(self):
        self._tfbuff = tf2.Buffer()
        self._lis = tf2.TransformListener(self._tfbuff)
        self._tf_static_broad = tf2.StaticTransformBroadcaster()
        self._broad = tf2.TransformBroadcaster()

    def _fillMsg(self, pos = [0,0,0], rot = [0,0,0,1] ,point_name ='', ref="map"):
        TS = TransformStamped()
        TS.header.stamp = rospy.Time.now()
        TS.header.frame_id = ref
        TS.child_frame_id = point_name
        TS.transform.translation.x = pos[0]
        TS.transform.translation.y = pos[1]
        TS.transform.translation.z = pos[2]
        TS.transform.rotation.x = rot[0]
        TS.transform.rotation.y = rot[1]
        TS.transform.rotation.z = rot[2]
        TS.transform.rotation.w = rot[3]
        return TS

    def pub_tf(self, pos = [0,0,0], rot = [0,0,0,1] ,point_name ='', ref="map"):
        dinamic_ts = self._fillMsg(pos, rot, point_name, ref)
        self._broad.sendTransform(dinamic_ts)

    def pub_static_tf(self, pos = [0,0,0], rot = [0,0,0,1] ,point_name ='', ref="map"):
        static_ts = self._fillMsg(pos, rot, point_name, ref)
        self._tf_static_broad.sendTransform(static_ts)

    def change_ref_frame_tf(self, point_name = '', new_frame = 'map'):
        try:
            traf = self._tfbuff.lookup_transform(new_frame, point_name, rospy.Time(0))
            translation, rotational = self.tf2_obj_2_arr(traf)
            self.pub_static_tf(pos = translation, rot = rotational, point_name = point_name, ref = new_frame)
            return True
        except:
            return False

    def getTF(self, target_frame='', ref_frame='map'):
        try:
            tf = self._tfbuff.lookup_transform(ref_frame, target_frame, rospy.Time(0))
            return self.tf2_obj_2_arr(tf)
        except:
            return [False,False]

    def tf2_obj_2_arr(self, transf):
        pos = []
        pos.append(transf.transform.translation.x)
        pos.append(transf.transform.translation.y)
        pos.append(transf.transform.translation.z)
    
        rot = []
        rot.append(transf.transform.rotation.x)
        rot.append(transf.transform.rotation.y)
        rot.append(transf.transform.rotation.z)
        rot.append(transf.transform.rotation.w)

        return [pos, rot]

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
        params["model_folder"] = usr_url+"Documentos/openpose/models/"
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
    
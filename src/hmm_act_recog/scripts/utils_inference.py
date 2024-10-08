#!/usr/bin/env python3
from cv_bridge import CvBridge
from sensor_msgs.msg import Image ,PointCloud2
from geometry_msgs.msg import TransformStamped,Point, Quaternion
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import rospy
import ros_numpy
import tf
import tf2_ros as tf2
import numpy as np
import cv2
import sys
from glob import glob
from math import ceil,floor
from os import path
from rospkg import RosPack
from utils_extras import *
from hmm_act_recog.msg import Floats
#---
try:
    usr_url=path.expanduser( '~' )
    # CHANGE THIS PATH TO WHERE OPENPOSE DIR WAS INSTALLED,
    #       'openpose/build/python' <-- do not change it, just the first part
    #sys.path.append(usr_url+'/openpose/build/python');
    sys.path.append(usr_url+'/openpose/build/python');
    from openpose import pyopenpose as op
    
except ImportError as e:
	print('Error: OpenPose library could not be found. Did you enable `BUILD_PYTHON` in CMake and have this Python script in the right folder?')
	raise e
#---


global rospack,bridge,tf_man
rospy.init_node('recognize_action_server')
rospack = RosPack()
bridge = CvBridge()

#========================================

def hmmActionRecognition(datum,opWrapper,response):
    print("Opcion 1: USB_CAM y no accion inferida, solo pose con Openpose")
    data = rospy.wait_for_message("/usb_cam/image_raw",Image,timeout=5) #
    cv2_img = bridge.imgmsg_to_cv2(data)
    image=np.copy(cv2_img)
    h,w,_=image.shape
    

    datum.cvInputData = image
    opWrapper.emplaceAndPop(op.VectorDatum([datum]))
    if datum.poseKeypoints is not None:
        dataout=np.copy(datum.poseKeypoints[0,:,:2])
        response.i_out=1
        image=draw_skeleton(dataout,h,w,image,cnt_person=0,bkground=True)
    else:
        print("NO SK DETECTADO")
        dataout = -1
    img_msg=bridge.cv2_to_imgmsg(image)
    if len(response.im_out.image_msgs)==0:
        response.im_out.image_msgs.append(img_msg)
    else:
        response.im_out.image_msgs[0]=img_msg

    return dataout,image

#---------------------------------------------------
def estimateArmTFforObjectGive(datum,opWrapper,response):
    print("Opcion 2, estimar brazo para dar objeto")
    cnt=0

    while True:

        im=rgbd.get_image()
        #################### USE WITH USB_CAM #############################
        #data = rospy.wait_for_message("/usb_cam/image_raw",Image,timeout=5) #
        #cv2_img = bridge.imgmsg_to_cv2(data)
        #im=np.copy(cv2_img)
        #######################################################################
        h,w,_=im.shape
        dataout=np.zeros((25,2))
        datum.cvInputData = im
        opWrapper.emplaceAndPop(op.VectorDatum([datum]))
        if datum.poseKeypoints is not None:
            cnt+=1
            dataout=np.copy(datum.poseKeypoints[0,:,:2])
            im=draw_skeleton(dataout,h,w,im,cnt_person=0,bkground=True)
            cv2.putText(img=im, text="Contador: "+str(cnt),org=(5, 20),fontFace=cv2.FONT_HERSHEY_SIMPLEX, 
                        fontScale=0.6, color=(35, 255, 148),thickness=2)
            if cnt==50:
                print("Obteniendo rgbd...")
                frameC,dataPC=get_rgbd_and_pointC()
                print("esqueleto encontrado")
                
                mano,codo=detect_pointing_arm(dataout,dataPC)
                tf_man.pub_static_tf(pos=codo,point_name='CODO',ref='head_rgbd_sensor_link')
                tf_man.pub_static_tf(pos=mano,point_name='MANO',ref='head_rgbd_sensor_link')
                rospy.sleep(0.8)
                tf_man.change_ref_frame_tf(point_name='CODO',new_frame='map')
                tf_man.change_ref_frame_tf(point_name='MANO',new_frame='map')
                rospy.sleep(0.8)
                print("tf publicada")

                response.i_out=1
                
                break
        else:
            dataout = -1    
    img_msg=bridge.cv2_to_imgmsg(im)
    if len(response.im_out.image_msgs)==0:
        response.im_out.image_msgs.append(img_msg)
    else:
        response.im_out.image_msgs[0]=img_msg
    return response

#---------------------------------------------------
def getPointingArm(datum,opWrapper,response):
    max_inf_it=30
    flo=Floats()
    print("Opcion 3, estimar brazo que esta apuntando")
    cnt=0
    while True:

        im=rgb.get_image()
        #im=cv2.cvtColor(im, cv2.COLOR_BGR2RGB)
        h,w,_=im.shape
        dataPC=rgbd.get_points()
        dataout=np.zeros((25,2))
        skeletons_xyz=np.zeros((25,3))
        datum.cvInputData = im
        opWrapper.emplaceAndPop(op.VectorDatum([datum]))
        if datum.poseKeypoints is not None:
            cnt+=1
            dataout=np.copy(datum.poseKeypoints[0,:,:2])
                
            if cnt==max_inf_it:
                flg=0
                print("Obteniendo rgbd...")
                # Para evitar lecturas nan y no retorne coordenadas nan
                while flg!=1:
                    frameC,dataPC=get_rgbd_and_pointC()
                    im=cv2.cvtColor(frameC, cv2.COLOR_BGR2RGB)
                    print("esqueleto encontrado")
                    # Calculo una ultima vez con openpose y la imagen que se obtiene de pointcloud
                    datum.cvInputData = im
                    opWrapper.emplaceAndPop(op.VectorDatum([datum]))
                    if datum.poseKeypoints is not None:
                        dataout=np.copy(datum.poseKeypoints[0,:,:2])# Y lo guardo
                    
                    im_t=draw_skeleton(dataout,h,w,im,cnt_person=0,bkground=True)
                    
                    mano,codo,f=detect_pointing_arm(dataout,dataPC)
                    print("MANO CODO",mano,codo)
                    if f!=-1:
                        tf_man.pub_static_tf(pos=codo,point_name='CODO',ref='head_rgbd_sensor_link')
                        tf_man.pub_static_tf(pos=mano,point_name='MANO',ref='head_rgbd_sensor_link')#print("cambiando referencia")
                        tf_man.change_ref_frame_tf(point_name='CODO',new_frame='map')
                        tf_man.change_ref_frame_tf(point_name='MANO',new_frame='map')
                        rospy.sleep(0.8)
                        manoM,_ = tf_man.getTF(target_frame='MANO',ref_frame='map')
                        codoM,_ = tf_man.getTF(target_frame='CODO',ref_frame='map')
                        ob_xyz = get_extrapolation(manoM,codoM)
                        flg=1
                    else:
                        print("HAY UNA NAN. Recalcula PC...")
                rospy.sleep(0.8)
                response.i_out=f
                break


    #print("DATA SK")
    #print(dataout)
    img_msg=bridge.cv2_to_imgmsg(im)
    
    flo.data=ob_xyz

    response.d_xyz=flo
    img_msg2=bridge.cv2_to_imgmsg(dataout)
    if len(response.im_out.image_msgs)==0:
        response.im_out.image_msgs.append(img_msg2)
    else:
        response.im_out.image_msgs[0]=img_msg
        
    return response	
	
#---------------------------------------------------
# para bebida (mal)
def detectDrinkingPoseWithOP(datum,opWrapper,response,n_people_max = 2):
    conteo_sin_bebida=np.zeros(n_people_max)
    max_drink_cnt=10
    m_no_p=10
    c_nor=10
    cnt_normal=0
    no_person=0
    flg_out=False
    while True:
        image,dataPC=get_rgbd_and_pointC()#print(image.shape)
        h,w,_=image.shape
        datum.cvInputData = image
        opWrapper.emplaceAndPop(op.VectorDatum([datum]))#print("Body keypoints: \n" + str(datum.poseKeypoints))
        if datum.poseKeypoints is not None:
            no_person=0
            dataout=np.copy(datum.poseKeypoints[:,:,:2])
            order=np.argsort(np.argsort(dataout[:,0,0]))
            for i in range(dataout.shape[0]):
                image=draw_skeleton(dataout[i],h,w,image,bkground=True)
                draw_text_bkgn(image,text="Person:"+str(i),pos=(int(dataout[i,0,0]), int(dataout[i,0,1])-40),
                        font_scale=1.3,text_color=(255, 255, 32))
                draw_rect_sk(dataout[i],image)
                if detect_drinking(dataout[i]):
                    conteo_sin_bebida[i]=0
                    cnt_normal+=1
                    draw_text_bkgn(image,text="Person "+str(i)+": ",pos=(int(dataout[i,0,0]), int(dataout[i,0,1])-40),
                                font_scale=1.3,text_color=(32, 255, 255))
                    draw_text_bkgn(image,text="Con bebida",pos=(int(dataout[i,0,0]), int(dataout[i,0,1])-20),
                                font_scale=1.3,text_color=(32, 255, 255))
                else:
                    cnt_normal=0
                    draw_text_bkgn(image,text="Person "+str(i)+":",pos=(int(dataout[i,0,0]), int(dataout[i,0,1])-40),
                                font_scale=1.3,text_color=(32, 255, 255))
                    draw_text_bkgn(image,text="Sin bebida",pos=(int(dataout[i,0,0]), int(dataout[i,0,1])-20),
                                font_scale=1.3,text_color=(32, 255, 255))
                    conteo_sin_bebida[i]+=1
        else:
            no_person+=1
        
        print(conteo_sin_bebida)# -----------------------

        if no_person==m_no_p:
            response.i_out=3
            break
        if cnt_normal==c_nor:
            print("TODOS CON BEBIDA DURANTE UN TIEMPO RAZONABLE")
            response.i_out=2
            break
        else:
            for c in range(n_people_max):
                if conteo_sin_bebida[c]==max_drink_cnt:
                    print("\n\nNO TIENE BEBIDA LA PERSONA :{}, PROCEDO A OFRECER UNA\n".format(c))
                    flg_out=True
                    head,f=return_xyz_sk(dataout,c,dataPC)
                    if f!=-1:
                        print(head)
                        response.i_out=1
                        tf_man.pub_static_tf(pos=head,point_name='head_xyz',ref='head_rgbd_sensor_link')
                        
                        tf_man.change_ref_frame_tf(point_name='head_xyz',new_frame='map')
                        rospy.sleep(0.8)
                        ob_xyz,_ = tf_man.getTF(target_frame='head_xyz',ref_frame='map')
                        print(ob_xyz)#flo.data=ob_xyz#response.d_xyz=flo
                        break
                    else: 
                        response.i_out=0
                        print("DATOS NAN, no se retorna datos")
            if flg_out:
                break
    return response,dataout,image

#---------------------------------------------------
def detectWavingRestaurant(datum,opWrapper,response):
    #print("OPCION 5 PARA RESTAURANT")
    counting=0
    conteoTOTAL=0
    while True:
        print("CONTEO",counting)
        print(f"Conteo GLOBAL:{conteoTOTAL}")
        points_msg=rospy.wait_for_message("/hsrb/head_rgbd_sensor/depth_registered/rectified_points",PointCloud2,timeout=5)
        points_data = ros_numpy.numpify(points_msg)
        image,maskedImage = removeBackground(points_msg,distance = 10)
        h,w,_=image.shape
        #dataout=np.zeros((25,2))
        datum.cvInputData = maskedImage
        opWrapper.emplaceAndPop(op.VectorDatum([datum]))
        if datum.poseKeypoints is None:
            
            print("No se encontro esqueleto/persona")
            response.i_out=-1
            img_msg=bridge.cv2_to_imgmsg(maskedImage)
            if len(response.im_out.image_msgs)==0:
                response.im_out.image_msgs.append(img_msg)
            else:
                response.im_out.image_msgs[0]=img_msg
            
            conteoTOTAL+=1
        
        else:
            #print("datum shape",datum.poseKeypoints.shape)
            dataout=np.copy(datum.poseKeypoints[:,:,:2])
            for i in range(dataout.shape[0]):
                maskedImage=draw_skeleton(dataout[i],h,w,maskedImage,bkground=True)
                draw_text_bkgn(maskedImage,text="Person "+str(i)+": ",pos=(int(dataout[i,0,0]), int(dataout[i,0,1])-40),
                                font_scale=1.3,text_color=(32, 255, 255))
                
            img_msg=bridge.cv2_to_imgmsg(maskedImage)
            if len(response.im_out.image_msgs)==0:
                response.im_out.image_msgs.append(img_msg)
            else:
                response.im_out.image_msgs[0]=img_msg
            sk_idx = detectWaving(dataout,maskedImage,points_msg)
            if sk_idx == -1:
                #print("NO PERSON WAVING")
                response.i_out=-1
                counting = 0
            else:
                counting += 1
                response.i_out=1
        
        conteoTOTAL += 1
        if counting >= 4:
            break

        if conteoTOTAL > 5:
            response.i_out = -1
            break
    print("RESPUESTA,",response.i_out)
    if response.i_out == -1: return response 
    #----	
    head_mean = np.concatenate((dataout[sk_idx,0:1,:],dataout[sk_idx,15:19,:]),axis=0)
    head_mean = np.sum(head_mean,axis=0)/np. count_nonzero(head_mean,axis=0)[0] 
    

    head_xyz =[points_data['x'][int(head_mean[1]), int(head_mean[0])],
                points_data['y'][int(head_mean[1]), int(head_mean[0])],
                points_data['z'][int(head_mean[1]), int(head_mean[0])]]
    print("HEAD XYZ of waving person", head_xyz)

    if head_xyz[0] is not None:
        print("PUBLICANDO....")
        tf_man.pub_static_tf(pos=head_xyz,point_name='person_waving',ref='head_rgbd_sensor_link')
        #tf_man.pub_static_tf(pos=[head_xyz[0],head_xyz[1],0],point_name='person_waving',ref='head_rgbd_sensor_link') # to z=0
        rospy.sleep(0.8)
        print("CAMBIANDO REF")
        tf_man.change_ref_frame_tf(point_name='person_waving',new_frame='map')
        rospy.sleep(0.8)
        response.i_out = 1
    else:
        print("No se pudo publicar")
        response.i_out = 2

    img_msg2=bridge.cv2_to_imgmsg(dataout)
    response.im_out.image_msgs.append(img_msg2)

    return response

#---------------------------------------------------
def useOpenPoseOnce(datum,opWrapper,response):
    print("Opcion distinta, una sola imagen con openpose")


    im=rgbd.get_image()
    dataout=np.zeros((25,2))
    datum.cvInputData = im
    opWrapper.emplaceAndPop(op.VectorDatum([datum]))
    if datum.poseKeypoints is not None:
        dataout=np.copy(datum.poseKeypoints[0,:,:2])
        print("Obteniendo rgbd...")
        frameC,dataPC=get_rgbd_and_pointC()
        print("esqueleto encontrado")#pub_points(dataPC,dataout,skPub=1)
        print("tf publicada")
        if dataout[0,0]!=0 and dataout[0,1]!=0:
            response.i_out=1
        else:
            print("Se detecto esqueleto pero no la cara de la persona")
            response.i_out=-1
    else:
        print("No se encontro esqueleto/persona")
        response.i_out=-1

    img_msg=bridge.cv2_to_imgmsg(im)
    img_msg2=bridge.cv2_to_imgmsg(dataout)
    if len(response.im_out.image_msgs)==0:
        response.im_out.image_msgs.append(img_msg)
    else:
        response.im_out.image_msgs[0]=img_msg

    return response

#---------------------------------------------------
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
class RGBD:
    def __init__(self, PC_rectified_point_topic = "/hsrb/head_rgbd_sensor/depth_registered/rectified_points"):
        self._br = tf.TransformBroadcaster()
        self._cloud_sub = rospy.Subscriber(PC_rectified_point_topic,
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

# Color segmentator
    def color_segmentator(self, color="orange"):
        image = self.get_image()
        if(color == "blue"):
            lower_threshold = (100, 120, 100)
            upper_threshold = (150, 220, 240)
        else:
            lower_threshold = (102, 95, 97)
            upper_threshold = (115, 255, 255)
        img_hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
        mask = cv2.inRange(img_hsv, lower_threshold, upper_threshold)
        res = cv2.bitwise_and(img_hsv, img_hsv, mask=mask)
        pos = []
        pixels = cv2.findNonZero(mask)
        pixels = list(cv2.mean(pixels))
        pos.append(pixels[:2])
        return pos

#---------------------------------------------------
class TF_MANAGER:
    def __init__(self):
        self._tfbuff = tf2.Buffer()
        self._lis = tf2.TransformListener(self._tfbuff)
        self._tf_static_broad = tf2.StaticTransformBroadcaster()
        self._broad = tf2.TransformBroadcaster()

    def _fillMsg(self, pos=[0, 0, 0], rot=[0, 0, 0, 1], point_name='', ref="map"):
        TS = TransformStamped()
        TS.header.stamp = rospy.Time.now()
        TS.header.frame_id = ref
        TS.child_frame_id = point_name
        TS.transform.translation = Point(*pos)
        TS.transform.rotation = Quaternion(*rot)
        return TS

    def pub_tf(self, pos=[0, 0, 0], rot=[0, 0, 0, 1], point_name='', ref="map"):
        dinamic_ts = self._fillMsg(pos, rot, point_name, ref)
        self._broad.sendTransform(dinamic_ts)

    def pub_static_tf(self, pos=[0, 0, 0], rot=[0, 0, 0, 1], point_name='', ref="map"):
        static_ts = self._fillMsg(pos, rot, point_name, ref)
        self._tf_static_broad.sendTransform(static_ts)

    def change_ref_frame_tf(self, point_name='', rotational=[0, 0, 0, 1], new_frame='map'):
        try:
            traf = self._tfbuff.lookup_transform(
                new_frame, point_name, rospy.Time(0))
            translation, _ = self.tf2_obj_2_arr(traf)
            self.pub_static_tf(pos=translation, rot=rotational,
                               point_name=point_name, ref=new_frame)
            return True
        except:
            return False

    def getTF(self, target_frame='', ref_frame='map'):
        try:
            tf = self._tfbuff.lookup_transform(
                ref_frame, target_frame, rospy.Time(0), rospy.Duration(1.5))
            return self.tf2_obj_2_arr(tf)
        except:
            return [False, False]

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
    
    # get the file path for rospy_tutorials
    # Para cargar los modelos en el codigo de HMM con Vit
    route=path.join(rospack.get_path("hmm_act_recog"))+"/scripts/models/"
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
def get_rgbd_and_pointC():

    #data=rospy.wait_for_message("/camera/depth/image_raw",PointCloud2)
    data=rospy.wait_for_message("/hsrb/head_rgbd_sensor/depth_registered/rectified_points",PointCloud2)
    #print("datos recibidos")
    # NO SE CORRIGE ( -> correct_points() ), SE OBSERVO QUE NO FUE NECESARIO
    np_data=ros_numpy.numpify(data)
    frame=np_data['rgb'].view((np.uint8, 4))[..., [2, 1, 0]]
    frame=cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    return frame,np_data

#---------------------------------------------------
def return_xyz_sk(lastSK,indx,cld_points):

    if lastSK[indx,0,0]==0 or lastSK[indx,0,1]==0:
        if lastSK[indx,17,0]!=0 and lastSK[indx,17,1]!=0:
            sk=17
        elif lastSK[indx,18,0]!=0 and lastSK[indx,18,1]!=0:
            sk=18
    else:
        sk=0
    print("SK:::",sk)
    


    head_xyz=[cld_points['x'][round(lastSK[indx,sk,1]), round(lastSK[indx,sk,0])],
            cld_points['y'][round(lastSK[indx,sk,1]), round(lastSK[indx,sk,0])],
            cld_points['z'][round(lastSK[indx,sk,1]), round(lastSK[indx,sk,0])]]

    if ~np.isnan(head_xyz).any():

        return head_xyz,1

    else:
        print("datos nan")
        #print(head_xyz)
        return -1,-1

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
def detect_drinking(data):
    """
        Brazo derecho: 2,3,4 (hombro,codo,mano) 
        Brazo izquierdo: 5,6,7 (hombro,codo,mano)
    """
    if data[2,:].any()!=0 or data[3,:].any()!=0 or data[4,:].any()!=0:
        cos_A=np.dot((data[2,:]-data[3,:]),(data[4,:]-data[3,:]))/(np.linalg.norm((data[2,:]-data[3,:]))*np.linalg.norm((data[4,:]-data[3,:])))
        ang_der=np.rad2deg(np.arccos(cos_A))
    else:
        ang_der=130
    
    if data[5,:].any()!=0 or data[6,:].any()!=0 or data[7,:].any()!=0:
        cos_B=np.dot((data[5,:]-data[6,:]),(data[7,:]-data[6,:]))/(np.linalg.norm((data[5,:]-data[6,:]))*np.linalg.norm((data[7,:]-data[6,:])))
        ang_izq=np.rad2deg(np.arccos(cos_B))
    else:
        ang_izq=130
        
    if abs(ang_der)<=120 or abs(ang_izq)<=120:
        #print("Se detecta que tiene una bebida")
        return True
    else:
        #print("No se detecta brazo con bebida")
        return False

#------------------------------------------
def draw_text_bkgn(img, text,
          font=cv2.FONT_HERSHEY_PLAIN,
          pos=(0, 0),
          font_scale=1,
          font_thickness=2,
          text_color=(0, 255, 0),
          text_color_bg=(0, 0, 0)
          ):

    x, y = pos
    text_size, _ = cv2.getTextSize(text, font, font_scale, font_thickness)
    text_w, text_h = text_size
    cv2.rectangle(img, pos, (x + text_w, y + text_h), text_color_bg, -1)
    cv2.putText(img, text, (x, y + text_h + int(font_scale) - 1), font, font_scale, text_color, font_thickness)

    return text_size

#------------------------------------------
def draw_rect_sk(dataout,im,mask=False):
    f=2
    color=(255,255,0)
    if mask:
        f=-1
        color=(255,255,255)
    
    bbox=get_rect_bbox(dataout,im)
    cv2.rectangle(im,bbox[0],bbox[-1],color,f)
    
    return im

#------------------------------------------
def get_rect_bbox(dataout,im):

    h,w,_=im.shape

    bbox=np.array([[floor(np.min(dataout[:,0][np.nonzero(dataout[:,0])])),floor(np.min(dataout[:,1][np.nonzero(dataout[:,1])]))],
                  [ceil(np.max(dataout[:,0][np.nonzero(dataout[:,0])])),ceil(np.max(dataout[:,1][np.nonzero(dataout[:,1])]))]
                 ])
    if bbox[0,0]>50:
        bbox[0,0]-=50

    if bbox[0,1]>50:
        bbox[0,1]-=50

    if w-bbox[1,0]>50:
        bbox[1,0]+=50
    else:
        bbox[1,0]+=(w-bbox[1,0]-1)
    if h-bbox[1,1]>50:
        bbox[1,1]+=50
    else:
        bbox[1,1]+=(h-bbox[1,1]-1)    
    return bbox

#------------------------------------------
def detectWaving(dataout,im,points_msg):
    #print(dataout)
    # El primero que detecte lo retorna
    for i,sk in enumerate(dataout):
        if (sk[4,1] <= sk[0,1] and sk[4,1]!= 0)  or (sk[7,1] <= sk[0,1] and sk[7,1]!= 0):
            return i

    return -1

#------------------------------------------
def removeBackground(points_msg,distance = 2):
    # Obtengo rgb
    points_data = ros_numpy.numpify(points_msg)
    image_data = points_data['rgb'].view((np.uint8, 4))[..., [2, 1, 0]]   
    image=cv2.cvtColor(image_data, cv2.COLOR_BGR2RGB)
    image = points_data['rgb'].view((np.uint8, 4))[..., [2, 1, 0]]
    rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)    

    # Quito todos los pixeles que esten a una distancia mayor y/o a una distancia menor
    # Para poder obtener una mascara con ceros y unos
    zs_no_nans=np.where(~np.isnan(points_data['z']),points_data['z'],10)
    img_corrected = np.where((zs_no_nans < distance + 0.3),zs_no_nans,0)
    #img_corrected = np.where((img_corrected >1.5),img_corrected,0)
    img_corrected = np.where((img_corrected == 0),img_corrected,1)

    # operacion AND entre la imagen original y la mascara para quitar fondo (background)
    #img_corrected = img_corrected.astype(np.uint8)
    masked_image = cv2.bitwise_and(rgb_image, rgb_image, mask=img_corrected.astype(np.uint8))
    return rgb_image, masked_image


tf_man = TF_MANAGER()

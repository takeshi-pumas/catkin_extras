#!/usr/bin/env python3


from cv_bridge import CvBridge
from act_recog.srv import Recognize,RecognizeResponse,RecognizeRequest
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
#import utils_inf as infHMM
#from utils_srv2 import *

#print(listdir('.'))
try:
    sys.path.append('../openpose/build/python');
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
class RGBD():
    u"""RGB-Dデータを扱うクラス"""

    def __init__(self):
        self._br = tf.TransformBroadcaster()
        # ポイントクラウドのサブスクライバのコールバックに_cloud_cbメソッドを登録
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
        # ポイントクラウドを取得する
        self._points_data = ros_numpy.numpify(msg)

        # 画像を取得する
        self._image_data = \
            self._points_data['rgb'].view((np.uint8, 4))[..., [2, 1, 0]]

        # 色相画像を作成する
        hsv_image = cv2.cvtColor(self._image_data, cv2.COLOR_RGB2HSV_FULL)
        self._h_image = hsv_image[..., 0]

        # 色相の閾値内の領域を抽出する
        self._region = \
            (self._h_image > self._h_min) & (self._h_image < self._h_max)

        # 領域がなければ処理を終える
        if not np.any(self._region):
            return

        # 領域からxyzを計算する
        (y_idx, x_idx) = np.where(self._region)
        x = np.average(self._points_data['x'][y_idx, x_idx])
        y = np.average(self._points_data['y'][y_idx, x_idx])
        z = np.average(self._points_data['z'][y_idx, x_idx])
        self._xyz = [y, x, z]
        #self._xyz = [x, y, z]

        # 座標の名前が設定されてなければ処理を終える
        if self._frame_name is None:
            return

        # tfを出力する
        self._br.sendTransform(
            (x, y, z), tf.transformations.quaternion_from_euler(0, 0, 0),
            rospy.Time(msg.header.stamp.secs, msg.header.stamp.nsecs),
            self._frame_name,
            msg.header.frame_id)

    def get_image(self):
        u"""画像を取得する関数"""
        return self._image_data

    def get_points(self):
        u"""ポイントクラウドを取得する関数"""
        return self._points_data

    def get_h_image(self):
        u"""色相画像を取得する関数"""
        return self._h_image

    def get_region(self):
        u"""抽出領域の画像を取得する関数"""
        return self._region

    def get_xyz(self):
        u"""抽出領域から計算されたxyzを取得する関数"""
        return self._xyz

    def set_h(self, h_min, h_max):
        u"""色相の閾値を設定する関数"""
        self._h_min = h_min
        self._h_max = h_max

    def set_coordinate_name(self, name):
        u"""座標の名前を設定する関数"""
        self._frame_name = name

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
#--------------------------------------------------------
def draw_skeleton(joints,hh,wh,im,norm=False,bkground=False,centroid=False):

    """
    conections 15 (of 25) joints:
        0-1   <-> centro cabeza         - cuello-torso
        0-11  <-> centro cabeza         - ojo D ?
        0-12  <-> centro cabeza         - ojo I ?
        1-2   <-> cuello-torso          - hombro D
        1-5   <-> cuello-torso          - hombro I
        1-8   <-> cuello-torso          - tronco-cadera ombligo
        2-3   <-> hombro D              - codo D
        3-4   <-> codo D                - muñeca D
        5-6   <-> hombro I              - codo I
        6-7   <-> codo I                - muñeca I
        8-9   <-> tronco-cadera ombligo - tronco-cadera D 
        8-10  <-> tronco-cadera ombligo -tronco-cadera I 
        11-13 <-> ojo D ?               - oreja D
        12-14 <-> ojo I ?               - oreja I

    conections 14 (of 18) joints:

        0-1  <-> centro cabeza  - cuello-torso
        0-10 <-> centro cabeza  - ojo D
        0-11 <-> centro cabeza  - ojo I
        1-2  <-> cuello-torso   - hombro D
        1-5  <-> cuello-torso   - hombro I
        1-8  <-> cuello-torso   - tronco-cadera D
        1-9  <-> cuello-torso   - tronco-cadera I
        2-3  <-> hombro D       - codo D
        3-4  <-> codo D         - muneca D
        5-6  <-> hombro I       - codo I
        6-7  <-> codo I         - muneca I
        10-12<-> ojo D          - oreja D
        11-13<-> ojo I          - oreja I
 

    """
    h=1
    w=1
    lineThick=2
    circleSize=3

    if norm:
        h=hh
        w=wh

    if bkground:
        bkgn=im.astype(np.uint8)
    else:
        bkgn=np.zeros((hh,wh,3),np.uint8)
    
    if centroid:
        lnCnt=int(joints.shape[0]/2)
        frame=np.zeros((lnCnt,2))
        frame[:,0]=joints[:lnCnt]
        frame[:,1]=joints[lnCnt:]
        if frame.shape[0]==15:
            conections=[[0,1],[0,11],[0,12],[1,2],[1,5],[1,8],
                [2,3],[3,4],[5,6],[6,7],[8,9],[8,10],
                [11,13],[12,14]]
        else:
            conections=[[0,1],[0,14],[0,15],[1,2],[1,5],[1,8],
                [1,11],[2,3],[3,4],[5,6],[6,7],[8,9],
                [9,10],[11,12],[12,13],[14,16],[15,17]]

        
        for conect in conections:
            if frame[conect[0]][0]!=0 and frame[conect[1]][1]!=0:
                bkgn=cv2.line(bkgn,(int(frame[conect[0]][0]*h),int(frame[conect[0]][1]*w)),(int(frame[conect[1]][0]*h),int(frame[conect[1]][1]*w)),(0,255,255),lineThick)
        for i in range(frame.shape[0]):
                    if frame[i][0]!=0.0 and frame[i][1]!=0.0:
                        bkgn=cv2.circle(bkgn,(int(frame[i][0]*h),int(frame[i][1]*w)),circleSize,(190,152,253),-1)

        return bkgn

    else:

        if joints.shape[0]==15:
            conections=[[0,1,0],[0,11,1],[0,12,2],[1,2,3],[1,5,4],[1,8,5],
                        [2,3,6],[3,4,7],[5,6,8],[6,7,9],[8,9,10],[8,10,11],
                        [11,13,12],[12,14,13]]
            # 0-1|0-11|0-12|1-2|1-5|1-8
            # 2-3|3-4|5-6|6-7|8-9|8-10 
            # 11-13|12-14
            colors=[(41,23,255),(99,1,249),(251,10,255),(10,75,255),(41,243,186),(10,10,255),   
                    (25,136,253),(40,203,253),(0,218,143),(0,218,116),(78,218,0),(253,183,31),   
                    (248,8,207),(248,8,76)]            

        elif joints.shape[0]==18:
            conections=[[0,1,0],[0,14,1],[0,15,2],[1,2,3],[1,5,4],[1,8,5],
                        [1,11,5],[2,3,6],[3,4,7],[5,6,8],[6,7,9],[8,9,10],
                        [9,10,11],[11,12,12],[12,13,13],[14,16,1],[15,17,1]]
            colors=[(253,45,31),(253,31,104),(253,31,184),(3,14,250),(15,104,252),(72,219,0),
                    (192,219,0),(18,170,255),(50,220,255),(50,255,152),(50,255,82),(113,219,0),
                    (167,251,77),(219,171,0),(219,113,0),(253,31,159),(159,31,253)]

        elif joints.shape[0]==25:
            conections=[[0,1,0],[0,15,1],[0,16,2],[1,2,3],[1,5,4],[1,8,5],
                        [2,3,6],[3,4,7],[5,6,8],[6,7,9],[8,9,10],[8,12,11],
                        [9,10,12],[10,11,13],[11,22,13],[11,24,13],[12,13,14],[13,14,15],
                        [14,19,15],[14,21,15],[15,17,16],[16,18,17],[19,20,15],[22,23,13]]
            # 0-1|0-15|0-16|1-2|1-5|1-8
            # 2-3|3-4|5-6|6-7|8-9|8-12 
            # 9-10|<10-11|11-22|11-24|22-23>|12-13|<13-14|14-19|14-21|19-20>|15-17|16-18
            colors=[(41,23,255),(99,1,249),(251,10,255),(10,75,255),(41,243,186),(10,10,255),
                    (25,136,253),(40,203,253),(0,218,143),(0,218,116),(78,218,0),(253,183,31),
                    (148,241,4),(239,255,1),(253,145,31),(253,80,31),(248,8,207),(248,8,76)]

        else:  #18 to less joints
            conections=[[0,1,0],[0,10,1],[0,11,2],[1,2,3],[1,5,4],[1,8,5],
                        [1,9,5],[2,3,6],[3,4,7],[5,6,8],[6,7,9],
                        [10,12,1],[11,13,1]]

            colors=[(253,45,31),(253,31,104),(253,31,184),(3,14,250),(15,104,252),(72,219,0),
                    (192,219,0),(18,170,255),(50,220,255),(50,255,152),(50,255,82),(113,219,0),
                    (167,251,77),(219,171,0),(219,113,0),(253,31,159),(159,31,253)]

        for i in range(joints.shape[0]):
            if joints[i][0]!=0.0 and joints[i][1]!=0.0:
                bkgn=cv2.circle(bkgn,(int(joints[i][0]*h),int(joints[i][1]*w)),circleSize,(255,255,255),-1)

        for conect in conections:
            if joints[conect[0]][0]!=0 and joints[conect[1]][1]!=0:
                bkgn=cv2.line(bkgn,(int(joints[conect[0]][0]*h),int(joints[conect[0]][1]*w)),(int(joints[conect[1]][0]*h),int(joints[conect[1]][1]*w)),colors[conect[2]],lineThick)

        return bkgn

#---------------------------------------------------
def init_openPose(n_people=1):
    try:
        params = dict()

        params["model_folder"] = "../openpose/models/"
        params["model_pose"] = "BODY_25"
        params["net_resolution"]="-1x208"
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
def correct_points(points_msg,low=.27,high=1000):
    #data = rospy.wait_for_message('/hsrb/head_rgbd_sensor/depth_registered/rectified_points', PointCloud2)
    np_data=ros_numpy.numpify(points_msg)
    trans,rot=tf_listener.lookupTransform('/map', '/head_rgbd_sensor_gazebo_frame', rospy.Time(0)) 
    
    eu=np.asarray(tf.transformations.euler_from_quaternion(rot))
    t=TransformStamped()
    rot=tf.transformations.quaternion_from_euler(eu[0],eu[1],eu[2])
    t.header.stamp = points_msg.header.stamp
    
    t.transform.rotation.x = rot[0]
    t.transform.rotation.y = rot[1]
    t.transform.rotation.z = rot[2]
    t.transform.rotation.w = rot[3]

    cloud_out = do_transform_cloud(points_msg, t)
    np_corrected=ros_numpy.numpify(cloud_out)
    #print(np_corrected.shape,np_data.shape)
    corrected=np_corrected.reshape(np_data.shape)

    return corrected

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

    

#--------------------------------------------------------
def callback(req):
    buf_size=35
    ctrlz=True
    buffer=[]
    cnt_acciones=0
    last_act=-1
    response=RecognizeResponse()
    
  
    #----------------
    if req.in_==1:
        print("Opcion 1\n\tObteniendo imagenes...")
        dataout=np.zeros((25,2))
        if ctrlz:
            cb2=centralizaSecuencia(cb,codebook=True)    
        else:
            cb2=cb
        while True:
            # <<<<<<<<<<<<<<<< obtengo imagen >>>>>>>>>>>>>>>>>>>>>
            im=rgb.get_image()
            imgOut=np.copy(im)
            #im=cv2.cvtColor(im, cv2.COLOR_BGR2RGB )

            #    Obtengo esqueleto
            #       Obtengo simbolo y append
            # <<<<<<<<<<<< obtengo esqueleto >>>>>>>>>>>>>>>>>>>>>>>>>
            datum.cvInputData = im
            opWrapper.emplaceAndPop(op.VectorDatum([datum]))
            if datum.poseKeypoints is not None:
                dataout=np.copy(datum.poseKeypoints[0,:,:2])
                # <<<< si detecto esqueleto, obtengo su simbolo y lo guardo >>>>>>>>
                symbol=create_vk(dataout,cb2,quitaJ=True,centralized=ctrlz)
                buffer.append(symbol)
            else:
                symbol=0
            
            
 
            
            if len(buffer)==buf_size+1:
                buffer.pop(0)
            #<<<<< si el buffer tiene un cierto tamaño, empieza la inferencia >>>>>>
            if len(buffer)==buf_size:
                probas=inf_Secuence(np.asarray(buffer),mA,mB,mPI)
                #print("Accion inferida: {}".format(class_names[np.argmax(probas[:])]))
                # <<<<<<< empieza a contar repeticiones de acciones inferidas >>>>>>
                if last_act==-1:
                    last_act=np.argmax(probas[:])

                else:
                    if np.argmax(probas[:])==last_act:
                        cnt_acciones+=1
                    else:
                        cnt_acciones=0
                        last_act=np.argmax(probas[:])
            
            im=draw_skeleton(dataout,h,w,im,bkground=True)
            im=draw_skeleton(cb[symbol],h,w,im,bkground=True,centroid=True)
            if last_act==-1:
                cv2.putText(img=im, text="buffer size:"+str(len(buffer))+", symbol det:"+str(symbol)+" reps:"+str(cnt_acciones), 
                org=(5, 20),fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.6, color=(35, 255, 148),thickness=2)
            else:
                cv2.putText(img=im, text="buffer size:"+str(len(buffer))+", symbol det:"+str(symbol)+" reps:"+str(cnt_acciones)+" act:"+str(class_names[last_act]), 
                org=(5, 20),fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.6, color=(35, 255, 148),thickness=2)
            if req.visual!=0:
                cv2.imshow("Imagen RGB",im)
                cv2.waitKey(10)

            #<<<<<<<<<<< si la inferencia se repite un determinado numero de veces
            #            termina el ciclo y regresa variables y demas >>>>>>>>>>>>
            
            if cnt_acciones==30 and last_act==4:
                print("Accion 4 detectada: {}".format(class_names[last_act]))
                img_msg=bridge.cv2_to_imgmsg(imgOut) # change to rgbd in robot
                sk_msg=bridge.cv2_to_imgmsg(dataout)
                print("Recibiendo imagen rgbd...")
                frameC,dataPC=get_coordinates()
                print("publicando...")
                frameC=draw_skeleton(dataout,h,w,frameC,bkground=True)  
                #if req.visual!=0:  
                #    cv2.imshow("IMAGEN RGBD",frameC)
                #    cv2.waitKey(400)
                pub_points(dataPC,dataout,skPub=1)
                print("tfs publicadas")

                break


            # Si es drink o neutral, sigue reconociendo hasta obtener otra accion
            elif cnt_acciones==30 and (last_act==2 or last_act==3):
                print("Accion detectada: neutral o drink, se sigue detectando... ")
                cnt_acciones=0
            
            # Si es waving, similar a pointing pero solo publica la cara
            elif cnt_acciones==30:
                print("Accion detectada: {}".format(class_names[last_act]))
                print("Recibiendo imagen rgbd...")
                frameC,dataPC=get_coordinates()
                print("publicando...")
                pub_points(dataPC,dataout,skPub=0)
                print("tf publicada")
                img_msg=bridge.cv2_to_imgmsg(imgOut) # change to rgbd in robot
                sk_msg=bridge.cv2_to_imgmsg(dataout)
                
                break
        if req.visual!=0:
            cv2.destroyAllWindows()
            cv2.waitKey(1)
     
        
        if len(response.im_out.image_msgs)==0:
            response.im_out.image_msgs.append(img_msg)
        else:
            response.im_out.image_msgs[0]=img_msg

        if len(response.sk.image_msgs)==0:
            response.sk.image_msgs.append(sk_msg)
        else:
            response.sk.image_msgs=sk_msg

        response.i_out=np.argmax(probas[:])

        return response
    #----------------
    # Para give object
    elif req.in_==2:
        print("Opcion 2, estimar brazo para dar objeto".format(req.in_))
        cnt=0
        while True:

            im=rgb.get_image()
            dataout=np.zeros((25,2))
            datum.cvInputData = im
            opWrapper.emplaceAndPop(op.VectorDatum([datum]))
            if datum.poseKeypoints is not None:
                cnt+=1
                dataout=np.copy(datum.poseKeypoints[0,:,:2])
                im=draw_skeleton(dataout,h,w,im,bkground=True)
                cv2.putText(img=im, text="Contador: "+str(cnt),org=(5, 20),fontFace=cv2.FONT_HERSHEY_SIMPLEX, 
                            fontScale=0.6, color=(35, 255, 148),thickness=2)

                if req.visual!=0:
                    cv2.imshow("Imagen RGB",im)
                    cv2.waitKey(10)
                print(cnt)
                if cnt==50:
                    
                    print("Obteniendo rgbd...")
                    frameC,dataPC=get_coordinates()
                    print("esqueleto encontrado")

                    mano,codo=detect_pointing_arm(dataout,dataPC)
                    tf_man.pub_static_tf(pos=codo,point_name='CODO',ref='head_rgbd_sensor_link')
                    tf_man.pub_static_tf(pos=mano,point_name='MANO',ref='head_rgbd_sensor_link')
                    #print("cambiando referencia")
                    tf_man.change_ref_frame_tf(point_name='CODO',new_frame='map')
                    tf_man.change_ref_frame_tf(point_name='MANO',new_frame='map')

                    print("tf publicada")
            
                    response.i_out=1
                    break
            
        if req.visual!=0:
            cv2.destroyAllWindows()
            cv2.waitKey(1)
    
        img_msg=bridge.cv2_to_imgmsg(im)
        img_msg2=bridge.cv2_to_imgmsg(dataout)
        if len(response.im_out.image_msgs)==0:
            response.im_out.image_msgs.append(img_msg)
        else:
            response.im_out.image_msgs[0]=img_msg

        if len(response.sk.image_msgs)==0:
            response.sk.image_msgs.append(img_msg2)
        else:
            response.sk.image_msgs[0]=img_msg2
        
        return response

    #----------------
    # Para obtener la imagen y esqueleto 1 vez y trabajar con ella fuera del servicio
    else:
        print("Opcion {}, una sola imagen con openpose".format(req.in_))
        

        im=rgb.get_image()
        dataout=np.zeros((25,2))
        datum.cvInputData = im
        opWrapper.emplaceAndPop(op.VectorDatum([datum]))
        if datum.poseKeypoints is not None:
            dataout=np.copy(datum.poseKeypoints[0,:,:2])
            print("Obteniendo rgbd...")
            frameC,dataPC=get_coordinates()
            print("esqueleto encontrado")
            #pub_points(dataPC,dataout,skPub=1)
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

        if len(response.sk.image_msgs)==0:
            response.sk.image_msgs.append(img_msg2)
        else:
            response.sk.image_msgs[0]=img_msg2
        
        return response    

#--------------------------------------------------------
def recognition_server():
    global tf_listener,rgb,rgbd, bridge,class_names,mA,mB,mPI,opWrapper,datum,cb,h,w,tf_man
    
    #---Parte para cargar lo necesario en inferencia con OpenPose y Markov---
    class_names=["wave_R","wave_L","neutral","drink","pointing"]
    mA,mB,mPI=loadModels(class_names)
    cb=np.load("src/act_recog/scripts/codebooks/codebook_LBG_160_s30.npy")
    opWrapper,datum=init_openPose()
    h=480
    w=640
    # ---
    
    rospy.init_node('recognize_action_server')
    rgb= RGB()
    rgbd=RGBD()
    bridge = CvBridge()
    tf_listener = tf.TransformListener()
    broadcaster= tf.TransformBroadcaster()
    tf_man=TF_MANAGER()
    tf_static_broadcaster= tf2.StaticTransformBroadcaster()
    rospy.loginfo("Action recognition service available")                    # initialize a ROS node
    s = rospy.Service('recognize_act', Recognize, callback) 
    print("Reconition service available")

    rospy.spin()


#========================================
if __name__ == "__main__":
    recognition_server()

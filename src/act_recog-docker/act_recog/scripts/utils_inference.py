#!/usr/bin/env python3
from cv_bridge import CvBridge
from sensor_msgs.msg import Image ,PointCloud2
from geometry_msgs.msg import  TransformStamped, Point, Quaternion
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
#---
try:
    sys.path.append('/openpose/build/python');
    #sys.path.append('/home/roboworks/openpose/build/python');
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
        TS.transform.translation = Point(*pos)
        TS.transform.rotation = Quaternion(*rot)
        return TS

    def pub_tf(self, pos = [0,0,0], rot = [0,0,0,1] ,point_name ='', ref="map"):
        dinamic_ts = self._fillMsg(pos, rot, point_name, ref)
        self._broad.sendTransform(dinamic_ts)

    def pub_static_tf(self, pos = [0,0,0], rot = [0,0,0,1] ,point_name ='', ref="map"):
        static_ts = self._fillMsg(pos, rot, point_name, ref)
        self._tf_static_broad.sendTransform(static_ts)

    def change_ref_frame_tf(self, point_name = '', rotational = [0,0,0,1], new_frame = 'map'):
        try:
            traf = self._tfbuff.lookup_transform(new_frame, point_name, rospy.Time(0))
            translation, _ = self.tf2_obj_2_arr(traf)
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
def init_openPose(n_people=-1,net_res="-1x208",model="BODY_25",heatmap=False):
    try:
        usr_url=path.expanduser( '~/' )
        params = dict()
		# OJO a este path, cambiarlo desde donde esté la carpeta openpose
        #print(usr_url+"openpose/models/")
        params["model_folder"] = "/openpose/models/"
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
    #print("datos recibidos")
    # NO SE CORRIGE ( -> correct_points() ), SE OBSERVO QUE NO FUE NECESARIO
    np_data=ros_numpy.numpify(data)
    frame=np_data['rgb'].view((np.uint8, 4))[..., [2, 1, 0]]
    frame=cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    return frame,np_data
#---------------------------------------------------

def return_xyz_sk(lastSK,indx,cld_points):
    sk=0
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
    if dataout[2,0]!=0 and dataout[3,0]!=0 and dataout[4,0]!=0:
        cos_A=np.dot((data[2,:]-data[3,:]),(data[4,:]-data[3,:]))/(0.0001+np.linalg.norm((data[2,:]-data[3,:]))*np.linalg.norm((data[4,:]-data[3,:])))
        ang_der=np.rad2deg(np.arccos(cos_A))
    else:
        ang_der=150

    if dataout[5,0]!=0 and dataout[6,0]!=0 and dataout[7,:].any()!=0:
        cos_B=np.dot((dataout[5,:]-dataout[6,:]),(dataout[7,:]-dataout[6,:]))/(0.0001+np.linalg.norm((dataout[5,:]-dataout[6,:]))*np.linalg.norm((dataout[7,:]-dataout[6,:])))
        ang_izq=np.rad2deg(np.arccos(cos_B))
    else:
        ang_izq=150    

    if (abs(ang_der)<=120 and abs(ang_izq)>120) or (abs(ang_izq)<=120 and abs(ang_der)>120):
        return True
    else:return False

#------------------------------------------

tf_man = TF_MANAGER()
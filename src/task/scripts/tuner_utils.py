#!/usr/bin/env python3

import smach
import smach_ros
from geometry_msgs.msg import PoseStamped, Point  , Quaternion , TransformStamped , Twist
from tf2_geometry_msgs import PointStamped
from std_srvs.srv import Trigger, TriggerResponse 
import moveit_commander
import moveit_msgs.msg
import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from object_classification.srv import *
from segmentation.srv import *
from human_detector.srv import Human_detector ,Human_detectorResponse 
from human_detector.srv import Point_detector ,Point_detectorResponse 
from ros_whisper_vosk.srv import GetSpeech
from face_recog.msg import *
from face_recog.srv import *
#import face_recognition 
from scipy.spatial import distance
import cv2  
import rospy 
import numpy as np
import actionlib
from hmm_navigation.msg import NavigateActionGoal, NavigateAction
from cv_bridge import CvBridge, CvBridgeError
import pandas as pd
from std_srvs.srv import Empty
from sensor_msgs.msg import Image , LaserScan , PointCloud2
import tf
import time

from nav_msgs.msg import OccupancyGrid
from hri_msgs.msg import RecognizedSpeech
from rospy.exceptions import ROSException
from vision_msgs.srv import *
import rospkg
import yaml
from act_recog.srv import Recognize,RecognizeResponse,RecognizeRequest
from object_classification.srv import *
from segmentation.srv import SegmentRegion, SegmentRegionRequest

from ros_whisper_vosk.srv import SetGrammarVosk

from utils.grasp_utils import *
from utils.misc_utils import *
from utils.nav_utils import *

global listener, broadcaster, tfBuffer, tf_static_broadcaster, scene, rgbd, head,train_new_face, wrist, human_detect_server, line_detector, clothes_color , head_mvit
global clear_octo_client, goal,navclient,segmentation_server  , tf_man , omni_base, brazo, speech_recog_server, bridge, map_msg, pix_per_m, analyze_face , arm , set_grammar
global recognize_action , classify_client, classify_client_dino , pointing_detect_server,placing_finder_server,hand_rgb 
##########################
global model , preprocess
rospy.init_node('tuner_node')
#head_mvit = moveit_commander.MoveGroupCommander('head')
#gripper =  moveit_commander.MoveGroupCommander('gripper')
#whole_body=moveit_commander.MoveGroupCommander('whole_body')

#broadcaster = tf.TransformBroadcaster()
#arm =  moveit_commander.MoveGroupCommander('arm')
tfBuffer = tf2_ros.Buffer()

listener = tf2_ros.TransformListener(tfBuffer)
broadcaster = tf2_ros.TransformBroadcaster()
tf_static_broadcaster = tf2_ros.StaticTransformBroadcaster()
clear_octo_client = rospy.ServiceProxy('/clear_octomap', Empty)   ###OGRASPING OBSTACLE 
human_detect_server = rospy.ServiceProxy('/detect_human' , Human_detector)  ####HUMAN FINDER OPPOSEBASED
pointing_detect_server = rospy.ServiceProxy('/detect_pointing' , Point_detector)
segmentation_server = rospy.ServiceProxy('/segment' , Segmentation)
placing_finder_server = rospy.ServiceProxy('/placing_finder' , Segmentation)


    ##### PLANE SEGMENTATION (PARALEL TO FLOOR)
navclient=actionlib.SimpleActionClient('/navigate', NavigateAction)   ### PUMAS NAV ACTION LIB
# scene = moveit_commander.PlanningSceneInterface()
speech_recog_server = rospy.ServiceProxy('/speech_recognition/vosk_service' ,GetSpeech)##############SPEECH VOSK RECOG FULL DICT
set_grammar = rospy.ServiceProxy('set_grammar_vosk', SetGrammarVosk)                   ###### Get speech vosk keywords from grammar (function get_keywords)         

recognize_face = rospy.ServiceProxy('recognize_face', RecognizeFace)                    #FACE RECOG
train_new_face = rospy.ServiceProxy('new_face', RecognizeFace)                          #FACE RECOG
analyze_face = rospy.ServiceProxy('analyze_face', RecognizeFace)    ###DEEP FACE ONLY
recognize_action = rospy.ServiceProxy('recognize_act', Recognize) 
classify_client = rospy.ServiceProxy('/classify', Classify)
#classify_client_dino = rospy.ServiceProxy('grounding_dino_detect', Classify_dino)
classify_client_dino = rospy.ServiceProxy('grounding_dino_detect', Classify_dino_receptionist) #beverage recognition
segment_service = rospy.ServiceProxy("segment_region", SegmentRegion)


#map_msg= rospy.wait_for_message('/augmented_map', OccupancyGrid , 20)####WAIT for nav pumas map .. 
#inflated_map= np.asarray(map_msg.data)
#img_map=inflated_map.reshape((map_msg.info.width,map_msg.info.height))
#pix_per_m=map_msg.info.resolution
#contours, hierarchy = cv2.findContours(img_map.astype('uint8'),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
#contoured=cv2.drawContours(img_map.astype('uint8'), contours, 1, (255,255,255), 1)

rgbd= RGBD()
hand_rgb = HAND_RGB()

bridge = CvBridge()
#segmentation_server = rospy.ServiceProxy('/segment_2_tf', Trigger) 
tf_man = TF_MANAGER()
gripper = GRIPPER()
#omni_base=OMNIBASE()        #  NAV ACTION
#omni_base=NAVIGATION()     #  nav UTILS
wrist= WRIST_SENSOR()
head = GAZE()
brazo = ARM()
line_detector = LineDetector()
# arm =  moveit_commander.MoveGroupCommander('arm')

#------------------------------------------------------

def detect_object_yolo(object_name,res):
    # find object_name in the response message from object_classification service (Yolo)
    for i,name in enumerate(res.names):
        if name.data[4:]==object_name:return res.poses[i]
    return False

#------------------------------------------------------
def seg_res_tf(res):
    #origin_map_img=[round(img_map.shape[0]*0.5) ,round(img_map.shape[1]*0.5)]   
    
    #brazo.set_named_target('go')
    if len(res.poses.data)==0:
        print('no objs')
        return False
    else:
        
        poses=np.asarray(res.poses.data)
        quats=np.asarray(res.quats.data)
        poses=poses.reshape((int(len(poses)/3) ,3     )      )  
        quats=quats.reshape((int(len(quats)/4) ,4     )      )  
        num_objs=len(poses)
        print(f'{num_objs} found')

        for i,cent in enumerate(poses):
            x,y,z=cent
            axis=[0,0,1]
            angle = tf.transformations.euler_from_quaternion(quats[i])[0]
            rotation_quaternion = tf.transformations.quaternion_about_axis(angle, axis)
            point_name=f'object_{i}'
            print (f'{point_name} found at {cent}')
            tf_man.pub_tf(pos=cent, rot =[0,0,0,1], point_name=point_name+'_norot', ref='map')## which object to choose   #TODO
            succ=tf_man.pub_tf(pos=cent, rot =rotation_quaternion, point_name=point_name, ref='map')## which object to choose   #TODO
            #tf_man.pub_static_tf(pos=cent, rot =[0,0,0,1], point_name=point_name+'_norot', ref='map')## which object to choose   #TODO
            #tf_man.pub_static_tf(pos=cent, rot =rotation_quaternion, point_name=point_name, ref='map')## which object to choose   #TODO
            rospy.sleep(0.5)                                                                        
            pose,_= tf_man.getTF(point_name)

            #print (f'Occupancy map at point object {i}-> pixels ',origin_map_img[1]+ round(pose[1]/pix_per_m),origin_map_img[0]+ round(pose[0]/pix_per_m), img_map[origin_map_img[1]+ round(pose[1]/pix_per_m),origin_map_img[0]+ round(pose[0]/pix_per_m)])
            ## Pixels from augmented image map server published map image
            """if img_map[origin_map_img[1]+ round(pose[1]/pix_per_m),origin_map_img[0]+ round(pose[0]/pix_per_m)]!=0:#### Yes axes seem to be "flipped" !=0:
                                                    print ('reject point suggested ( for floor), most likely part of arena, occupied inflated map')
                                                    #tf_man.pub_static_tf(pos=[0,0,0], point_name=point_name, ref='head_rgbd_sensor_rgb_frame')
                                                    #num_objs-=1"""
            
            print (f"object found at map coords.{pose} ")
    return succ
#------------------------------------------------------
def get_robot_px():
    trans, rot=tf_man.getTF('base_link')
    robot=np.asarray(trans[:2])
    
    return np.asarray((robot/pix_per_m).round(),dtype='int')

#------------------------------------------------------
def check_point_map(x,y):

    # Quiero saber punto 0,-7 m esta libre?
    ##meters, map (not pixels)
    #x,y=0.2,8
    #px=check_point_map(x,y)  # px is closest safe goal
    #px*pix_per_m
    xrob,yrob=get_robot_px()
    safe_xy=np.asarray((x,-1*y))
    safe_xy_px=point_to_px (x,y)
    delta_px=  safe_xy_px-np.asarray((xrob,yrob))
    delta_px= delta_px / np.linalg.norm(delta_px).round()
    delta_px=np.round(delta_px*10)
    newxy=np.zeros(2)
    
    for i in range(9):
        if (contoured[1024+ safe_xy_px[1].astype('int'),1024-safe_xy_px[0].astype('int')]!=0):   ## axis are twisted cause y sign
                    print ('not safe at', safe_xy_px , safe_xy_px*pix_per_m)
                    
                    xrob,yrob=get_robot_px()
                    delta_px=  safe_xy_px-np.asarray((xrob,yrob))
                    delta_px= delta_px / np.linalg.norm(delta_px).round()
                    delta_px=np.round(delta_px*10)
                    newxy[0]=safe_xy_px[0]-delta_px[0].astype('int')
                    newxy[1]=safe_xy_px[1]-delta_px[1].astype('int')
                    safe_xy_px=newxy
        else:
                print ('safe at', safe_xy_px, safe_xy_px*pix_per_m)
                return safe_xy_px
    return safe_xy_px
    
#------------------------------------------------------
def point_to_px(x,y):
    safe_xy=np.asarray((x,y))
    return np.round(safe_xy/pix_per_m).astype('int')
def px_to_point(px,py):
    return np.asarray((px,py))*pix_per_m

#------------------------------------------------------
def train_face(image, name):
    req=RecognizeFaceRequest()
    strings=Strings()
    string_msg= String()
    string_msg.data=name
    req.Ids.ids.append(string_msg)

    img_msg=bridge.cv2_to_imgmsg(image)
    req.in_.image_msgs.append(img_msg)
    res=train_new_face(req)
    
    return res.Ids.ids[0].data.split(' ')[0] == 'trained'

    #------------------------------------------------------
def wait_for_face(timeout=10 , name=''):
    
    rospy.sleep(0.3)
    
    start_time = rospy.get_time()
    strings=Strings()
    string_msg= String()
    string_msg.data='Anyone'
    while rospy.get_time() - start_time < timeout:
        img=rgbd.get_image()  
        req=RecognizeFaceRequest()
        print ('Got  image with shape',img.shape)
        req.Ids.ids.append(string_msg)
        img_msg=bridge.cv2_to_imgmsg(img)
        req.in_.image_msgs.append(img_msg)

        res= recognize_face(req)


        #NO FACE FOUND
        if res.Ids.ids[0].data == 'NO_FACE':
            print ('No face FOund Keep scanning')
            
            return None, None
        #AT LEAST ONE FACE FOUND
        else:
            print('at least one face found')
            ds_to_faces=[]
            for i , idface in enumerate(res.Ids.ids):
                print (i,idface.data)
                ds_to_faces.append(res.Ds.data[i])    ##
                if (idface.data)==name :
                    new_res= RecognizeFaceResponse()
                    new_res.Ds.data= res.Ds.data[i]
                    new_res.Angs.data= res.Angs.data[i:i+4]
                    new_res.Ids.ids=res.Ids.ids[i].data
                    print('return res,img',new_res)
                    print ('hit',idface.data, 'at' , res.Ds.data[i]  , 'meters')
                    ds_to_faces=[]
                    return new_res , img

            if len (ds_to_faces)!=0:
                i=np.argmin(ds_to_faces)
                new_res= RecognizeFaceResponse()
                new_res.Ds.data= res.Ds.data[i]
                new_res.Angs.data= res.Angs.data[i:i+4]
                new_res.Ids.ids=res.Ids.ids[i].data
                print('return res,img',new_res)
                ds_to_faces=[]
                return new_res , img





#------------------------------------------------------
#def wait_for_face(timeout=10):
#    
#    rospy.sleep(0.3)
#    
#    start_time = rospy.get_time()
#    strings=Strings()
#    string_msg= String()
#    string_msg.data='Anyone'
#    while rospy.get_time() - start_time < timeout:
#        img=rgbd.get_image()  
#        req=RecognizeFaceRequest()
#        print ('Got  image with shape',img.shape)
#        req.Ids.ids.append(string_msg)
#        img_msg=bridge.cv2_to_imgmsg(img)
#        req.in_.image_msgs.append(img_msg)
#
#        res= recognize_face(req)
#
#        if res.Ids.ids[0].data == 'NO_FACE':
#            print ('No face FOund Keep scanning')
#            return None, None
#        
#        else:return res , img
##------------------------------------------------------
def wait_for_push_hand(time=10):

    start_time = rospy.get_time()
    time= 10
    print('timeout will be ',time,'seconds')
    while rospy.get_time() - start_time < time:
        torque = wrist.get_torque()
        if np.abs(torque[1])>1.0:
            print(' Hand Pused Ready TO start')
            #takeshi_talk_pub.publish(string_to_Voice())
            talk('Im ready to start')
            return True
            break

    if (rospy.get_time() - start_time >= time):
        print(time, 'secs have elapsed with no hand push')
        return False
#
##------------------------------------------------------
def analyze_face_from_image(cv2_img,name=''):   
    #USING DEEP FACE SERVICE
    # FIND SOME CHARACTERISTICS FROM A FACE IMAGE
    req=RecognizeFaceRequest()
    strings=Strings()
    string_msg= String()
    string_msg.data='analyze '
    req.Ids.ids.append(string_msg)
    img_msg=bridge.cv2_to_imgmsg(cv2_img)
    req.in_.image_msgs.append(img_msg)
    #res=recognize(req)
    res = analyze_face(req)
    results=[]
    for chars in res.Ids.ids:
        results.append(chars.data)
    if (len(results)==1):return ''  ###NO FACE
    #name= 'Jack'
    pronoun='she'
    gender = results[0]
    age = results[-1]
    state = results[2]
    race = results[1]
    if gender=='Man':pronoun='he'
    takeshi_line = f'{name} has arrived... {pronoun} is a {gender}... I believe {pronoun}.' 
    takeshi_line += f'is  around  {age} years old... I would say he is a bit  {state}.'
    takeshi_line += f'And I might guess {pronoun} is of {race} descent.'
    return takeshi_line

#------------------------------------------------------
def bbox_3d_mean(points,boundRect):
    #Gets the mean of the Points from the pincloud enclosed in the bound Rect of the RGBD image

    xyz=[]
    xyz_n=points[['x','y','z']][boundRect[0]:boundRect[2],boundRect[3]:boundRect[1]]
    for i in range(xyz_n.shape[0]):
        for j in range(xyz_n.shape[1]):
            if ~np.isnan(xyz_n['x'][i,j]) and ~np.isnan(xyz_n['y'][i,j]) and ~np.isnan(xyz_n['z'][i,j]):
                xyz.append(np.asarray([xyz_n['x'][i,j],xyz_n['y'][i,j],xyz_n['z'][i,j]]))
                
    if len(xyz)!=0:
        return np.asarray(xyz).mean(axis=0)
    else:
        return np.ones(3)



#------------------------------------------------------
def read_yaml(known_locations_file='/known_locations.yaml'):
    rospack = rospkg.RosPack()
    file_path = rospack.get_path('config_files') + known_locations_file

    with open(file_path, 'r') as file:
        content = yaml.safe_load(file)
    return content
#----------------------------------------------------------
def yaml_to_df(known_locations_file='/known_locations.yaml'):
    con = read_yaml(known_locations_file)
    values=[]
    locations=[]
    for c in con:
        locations.append(c)

        for i in range(len(con[c])):
            values.append(list(con[c][i].values())[0])

    data=np.asarray(values).reshape((int(len(values)/7),7))    #x , y ,theta  ,quat   since z always 0
    df= pd.DataFrame( data)
    df.columns=['x','y','th','qx','qy','qz','qw']
    df['child_id_frame']=locations
    return df





#------------------------------------------------------
def read_tf(t):
    # trasnform message to np arrays
    pose=np.asarray((
        t.transform.translation.x,
        t.transform.translation.y,
        t.transform.translation.z
        ))
    quat=np.asarray((
        t.transform.rotation.x,
        t.transform.rotation.y,
        t.transform.rotation.z,
        t.transform.rotation.w
        ))
    
    return pose, quat
#------------------------------------------------------

def gaze_to_face():

    return False

#------------------------------------------------------
def get_favorite_drink_location(favorite_drink):
    bridge = CvBridge()
    # Convert image to ROS format
    #pointcloud_msg = rospy.wait_for_message("/hsrb/head_rgbd_sensor/depth_registered/rectified_points", PointCloud2)
    #pointcloud_msg = rgbd.get_points()
    print("mensaje nube de puntos")
    img_msg = rospy.wait_for_message('/hsrb/head_rgbd_sensor/rgb/image_raw', Image, timeout=5)
    print("mensaje image")
    img = bridge.imgmsg_to_cv2(img_msg,"bgr8")
    print(img.shape)
    #if pointcloud_msg is None:
    #    rospy.logerr("No se recibió la nube de puntos.")
    #    return
    #rospy.loginfo("Nube de puntos recibida.")
    # # Nombre de la región a segmentar (ajustar según el YAML)
    # region_name = "beverage_area"
    # rospy.wait_for_service("segment_region")
    # try:
    #     request = SegmentRegionRequest(pointcloud=pointcloud_msg, region_name=region_name)
    #     response = segment_service(request)

    #     if response.success:
    #         bridge = CvBridge()
    #         mask = bridge.imgmsg_to_cv2(response.mask, "mono8")
    #         # **Aplicar operaciones morfológicas para reducir ruido**
    #         kernel = np.ones((13, 13), np.uint8)  # Define un kernel de 5x5 (ajustable)
    #         mask = cv2.dilate(mask, kernel, iterations=4)  # **Rellena huecos**
    #         #mask = cv2.erode(mask, kernel, iterations=1)  # **Reduce pequeños artefactos**
    #         segment_img = cv2.bitwise_and(img, img, mask=mask)
    #         cv2.imwrite("img_debug.png",segment_img)
    #     else:
    #         rospy.logwarn("Error en segmentación: " + response.message)
    # except rospy.ServiceException as e:
    #     rospy.logerr("Error llamando al servicio: %s" % e)
    # print("Message Received")
    #ros_image=bridge.cv2_to_imgmsg(segment_img,encoding="bgr8")
    prompt_msg = String()
    prompt_msg.data = favorite_drink

    rospy.wait_for_service('grounding_dino_detect')
    try:
        response = classify_client_dino(img_msg, prompt_msg)
        print("Result:", response.result.data,"for drink:",favorite_drink)
        if response.result.data == "not found":
            res = False
        else: 
            res = True
        return res,response.result.data

    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")
        return False,e
#------------------------------------------------------
def detect_human_to_tf():
    humanpose=human_detect_server.call()
    print (humanpose)
    if (np.asarray((humanpose.x,humanpose.y,humanpose.z)).all()== np.zeros(3).all()):
        print (np.asarray((humanpose.x,humanpose.y,humanpose.z)))
        return False
    else:
        tf_man.pub_static_tf(np.asarray((humanpose.x,humanpose.x,humanpose.z)),point_name='human', ref='head_rgbd_sensor_link')
        succ=tf_man.change_ref_frame_tf('human')
        return succ
#------------------------------------------------------
def check_bag_hand_camera(imagen, x=220, y=265, w=45, h=15, bins=12,umbral = 0.99):
    print("hand_camera checking")
    
    # Recortar la región seleccionada
    recorte = imagen[y:y+h, x:x+w]

    # Calcular y cuantizar hist_actual
    hist_actual = {}
    colores = ('b', 'g', 'r')
    bin_size = 256 // bins

    for i, color in enumerate(colores):
        hist = cv2.calcHist([recorte], [i], None, [256], [0, 256])
        
        # Cuantización a 12 dimensiones
        hist_cuantizado = [
            int(sum(hist[j:j+bin_size])) 
            for j in range(0, 256, bin_size)
        ]

        # Asegúrate de que siempre tenga exactamente 12 elementos
        if len(hist_cuantizado) > bins:
            hist_cuantizado = hist_cuantizado[:bins]

        hist_actual[color] = hist_cuantizado
    print("Comparing...")
    hist_mano_vacia = {
        'b': [580, 50, 44, 51, 25, 0, 0, 0, 0, 0, 0, 0], 
        'g': [569, 59, 53, 69, 0, 0, 0, 0, 0, 0, 0, 0], 
        'r': [556, 70, 61, 63, 0, 0, 0, 0, 0, 0, 0, 0]

    }
    total_similaridad = 0
    total_canales = 0

    for color in ('b', 'g', 'r'):
        vacio = np.array(hist_mano_vacia[color]).astype(float)
        actual = np.array(hist_actual[color]).astype(float)

        # Verifica que ambos tengan 12 elementos
        if len(vacio) != 12 or len(actual) != 12:
            print(f"❌ Error: Los hist_actual para '{color}' no tienen 12 elementos")
            return "Error en los hist_actual"

        # Calcular la similaridad
        similaridad = 1 - distance.cosine(vacio, actual)
        print(f"✅ Similaridad para {color}: {similaridad:.4f}")

        total_similaridad += similaridad
        total_canales += 1

    promedio_similaridad = total_similaridad / total_canales
    print(f"📊 Promedio de similaridad: {promedio_similaridad:.4f}")

    # Decisión basada en el umbral
    if promedio_similaridad > umbral:
        return False  # Mano sin objeto
    else:
        return True  # Mano con objeto
#------------------------------------------------------
def comparar_hist_actual(hist_actual, umbral=0.9):
    print("Comparing...")
    hist_mano_vacia = {
        'b': [580, 50, 44, 51, 25, 0, 0, 0, 0, 0, 0, 0], 
        'g': [569, 59, 53, 69, 0, 0, 0, 0, 0, 0, 0, 0], 
        'r': [556, 70, 61, 63, 0, 0, 0, 0, 0, 0, 0, 0]

    }

    total_similaridad = 0
    total_canales = 0

    for color in ('b', 'g', 'r'):
        vacio = np.array(hist_mano_vacia[color]).astype(float)
        actual = np.array(hist_actual[color]).astype(float)

        # Verifica que ambos tengan 12 elementos
        if len(vacio) != 12 or len(actual) != 12:
            print(f"❌ Error: Los hist_actual para '{color}' no tienen 12 elementos")
            return "Error en los hist_actual"

        # Calcular la similaridad
        similaridad = 1 - distance.cosine(vacio, actual)
        print(f"✅ Similaridad para {color}: {similaridad:.4f}")

        total_similaridad += similaridad
        total_canales += 1

    promedio_similaridad = total_similaridad / total_canales
    print(f"📊 Promedio de similaridad: {promedio_similaridad:.4f}")

    # Decisión basada en el umbral
    if promedio_similaridad > umbral:
        return False  # Mano sin objeto
    else:
        return True  # Mano con objeto
    #------------------------------------------------------

def get_keywords_speech(timeout=5):
    try:
        msg = rospy.wait_for_message('/speech_recognition/final_result', String, timeout)
        result = msg.data
        return result
            
    except ROSException:
        rospy.loginfo('timeout')
        return 'timeout'

def check_room_px(px_pose,living_room_px_region,kitchen_px_region,bedroom_px_region,dining_room_px_region ):

    
    for i in range(4):
        if i==0:
            px_region=living_room_px_region
            region='living_room'
        if i==1:
            px_region=kitchen_px_region
            region='kitchen'
        if i==2:
            px_region=bedroom_px_region
            region='bedroom'
        if i==3:
            px_region=dining_room_px_region
            region='dining_room'
        #print (region,px_region,px_pose)
        if (px_pose[1]< px_region[1,1]) and (px_pose[1]> px_region[0,1]) and (px_pose[0]> px_region[0,0]) and (px_pose[0]< px_region[1,0]) : 
            print (f'in  {region}')
            return region

def detect_object_yolo(object_name,res):
    # find object_name in the response message from object_classification service (Yolo)
    objs=[]
    poses=[]
    for i,name in enumerate(res.names):
        if res.poses[i].position.x is not np.nan :
            objs.append(name.data[4:])
            poses.append(res.poses[i])
        if name.data[4:]==object_name:return name.data[4:], res.poses[i]
    if object_name=='all':
        print (objs, poses)
        return objs , poses
    return [],[]

#-----------------------------------------------------------------
def save_image(img,name='',dirName=''):
    rospack = rospkg.RosPack()
    file_path = rospack.get_path('images_repos')
    
    num_data = len(glob(path.join(file_path,"src",dirName,"*"))) if dirName else len(glob(path.join(file_path,"src","*")))
    
    num_data = str(num_data+1).zfill(4)

    name = "/" + name if (name and not(name.startswith("/"))) else name
    dirName = "/" + dirName if (dirName and not(dirName.startswith("/"))) else dirName

 
    if name and dirName:
        #print(file_path+"/src"+dirName+name+".jpg")
        cv2.imwrite(file_path+"/src"+dirName+name+num_data+".jpg",img)
    
    elif dirName and not(name):
        #print(file_path+"/src"+dirName+"/"+"image"+".jpg")
        cv2.imwrite(file_path+"/src"+dirName+"/"+"image"+num_data+".jpg",img)

    elif not(dirName) and name:
        #print(file_path+"/src"+name+".jpg")
        cv2.imwrite(file_path+"/src"+name+num_data+".jpg",img)
    
    else:
        #print(file_path+"/src"+"tmp"+".jpg")
        cv2.imwrite(file_path+"/src"+"image"+".jpg",img)
    
###################################################

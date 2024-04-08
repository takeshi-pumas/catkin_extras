#! /usr/bin/env python3
import tf
import cv2
import rospy  
import tf2_ros                                    
from human_detector.srv import Point_detector ,Point_detectorResponse 
from human_detector.srv import Human_detector ,Human_detectorResponse 

from cv_bridge import CvBridge
from object_classification.srv import *
import tf2_ros    
from segmentation.msg import *
import numpy as np
import ros_numpy
import os
import matplotlib.pyplot as plt
import cv2 
from collections import Counter
from sensor_msgs.msg import Image , LaserScan , PointCloud2
from geometry_msgs.msg import TransformStamped, Pose
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
#from utils.misc_utils import TF_MANAGER

#-----------------------------------------------------------------
global tf_listener, ptcld_lis, broadcaster , bridge , net , b_tf, b_st


rospy.init_node('human_pointing_detector') 
tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)
b_tf=tf2_ros.TransformBroadcaster()
b_st=tf2_ros.StaticTransformBroadcaster()
_tfbuff = tf2_ros.Buffer()

usr_url=os.path.expanduser( '~' )
protoFile = usr_url+"/openpose/models/pose/body_25/pose_deploy.prototxt"
weightsFile = usr_url+"/openpose/models/pose/body_25/pose_iter_584000.caffemodel"
net = cv2.dnn.readNetFromCaffe(protoFile, weightsFile)
#tf_listener = tf.TransformListener()
#broadcaster= tf.TransformBroadcaster()
#tf_static_broadcaster= tf2_ros.StaticTransformBroadcaster()
#pub = rospy.Publisher('/segmented_images', Image, queue_size=1)
bridge=CvBridge()

#-----------------------------------------------------------------
def write_tf(pose, q, child_frame , parent_frame='map'):
    t= TransformStamped()
    t.header.stamp = rospy.Time.now()
    #t.header.stamp = rospy.Time(0)
    t.header.frame_id =parent_frame
    t.child_frame_id =  child_frame
    t.transform.translation.x = pose[0]
    t.transform.translation.y = pose[1]
    t.transform.translation.z = pose[2]
    #q = tf.transformations.quaternion_from_euler(eu[0], eu[1], eu[2])
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]
    return t
#-----------------------------------------------------------------
def read_tf(t):
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
#-----------------------------------------------------------------

def getTF(target_frame='', ref_frame='map'):
        try:
            tf = tfBuffer.lookup_transform(
                ref_frame, target_frame, rospy.Time(0), rospy.Duration(1.5))
            return tf2_obj_2_arr(tf)
        except:
            return [False, False]
#-----------------------------------------------------------------
def tf2_obj_2_arr(transf):
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
#-----------------------------------------------------------------
def change_ref_frame_tf(point_name='', rotational=[0, 0, 0, 1], new_frame='map'):
        try:
            traf = tfBuffer.lookup_transform(
                new_frame, point_name, rospy.Time(0))
            translation, _ = tf2_obj_2_arr(traf)
            print("TRANSLATION",translation)
            t=write_tf(pose=translation,q=rotational,child_frame=point_name,parent_frame=new_frame)
            b_st.sendTransform(t)
            return True
        except:
            return False
#-----------------------------------------------------------------

#-----------------------------------------------------------------
def probmap_to_3d_mean(points_data,probMap, thres_prob=0.3):
    ##Prob map to 3d point

    mask=np.where(probMap>thres_prob)
    npmask=np.asarray(mask).T

    npmask.shape
    xyz=[]
    if len (npmask)>1:
        for a in npmask:
            ix,iy=a[0],a[1]
            aux=(np.asarray((points_data['x'][ix,iy],points_data['y'][ix,iy],points_data['z'][ix,iy])))
            #print (aux)
            if np.isnan(aux[0]) or np.isnan(aux[1]) or np.isnan(aux[2]):
                    'reject point'
            else:
                xyz.append(aux)

    xyz=np.asarray(xyz)
    if (len(xyz)!=0):
        cent=xyz.mean(axis=0)
    else:
        cent=np.zeros(3)
    return cent


def detect_human(points_msg):
    points_data = ros_numpy.numpify(points_msg)    
    image_data = points_data['rgb'].view((np.uint8, 4))[..., [2, 1, 0]]   
    image=cv2.cvtColor(image_data, cv2.COLOR_BGR2RGB)
    #image = points_data['rgb'].view((np.uint8, 4))[..., [2, 1, 0]]
    #rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    print (image.shape)
    frame=image
    inHeight = frame.shape[0]
    inWidth = frame.shape[1]


    # Prepare the frame to be fed to the network
    inpBlob = cv2.dnn.blobFromImage(frame, 1.0 / 255, (inWidth, inHeight), (0, 0, 0), swapRB=False, crop=False)

    # Set the prepared object as the input blob of the network
    net.setInput(inpBlob)

    output = net.forward()
    i = 0 #Face
    #i = 1# Neck
    probMap = output[0, i, :, :]
    probMap = cv2.resize(probMap, (inWidth, inHeight))
    cent= probmap_to_3d_mean(points_data,probMap)
    print (cent)
    if np.isnan(cent.any()):return Human_detectorResponse()
    print (cent)
    if np.isnan(cent.any()):cent=np.zeros(3)
    res=Human_detectorResponse()
    res.x= cent[0]
    res.y= cent[1]
    res.z= cent[2]
    
    return res    


def detect_pointing(points_msg):
    points_data = ros_numpy.numpify(points_msg)    
    image_data = points_data['rgb'].view((np.uint8, 4))[..., [2, 1, 0]]   
    image=cv2.cvtColor(image_data, cv2.COLOR_BGR2RGB)
    pts= points_data
    print (image.shape)
    frame=image
    inHeight = frame.shape[0]
    inWidth = frame.shape[1]
    # Prepare the frame to be fed to the network
    inpBlob = cv2.dnn.blobFromImage(image, 1.0 / 255, (inWidth, inHeight), (0, 0, 0), swapRB=False, crop=False)
    # Set the prepared object as the input blob of the network
    net.setInput(inpBlob)
    output = net.forward()
    thresh= 0.45
    poses=[]
    deb_imgr=image[:,:,0]
    deb_imgg=image[:,:,1]
    deb_imgb=image[:,:,2]
    res=Point_detectorResponse()
    for i in np.asarray((3,4,6,7)):
        probMap = output[0, i, :, :]
        probMap = cv2.resize(probMap, (inWidth, inHeight))
        if len(np.where(probMap>=thresh)[0]) ==0: pose=[0,0,0]  
        else:pose=  [np.nanmean(pts['x'][np.where(probMap>=thresh)]),
                     np.nanmean(pts['y'][np.where(probMap>=thresh)]),
                     np.nanmean(pts['z'][np.where(probMap>=thresh)])]
        poses.append(pose)
        #deb_img= probMap+ deb_img*0.3  #DEBUG IMAGE
        deb_imgr[np.where(probMap>=thresh)]= 255
        deb_imgg[np.where(probMap>=thresh)]= 255
        deb_imgb[np.where(probMap>=thresh)]= 255
    deb_img_rgb=cv2.merge((deb_imgr, deb_imgg, deb_imgb))
    res.debug_image.append(bridge.cv2_to_imgmsg(deb_img_rgb))
    #right elbow       ####
    # right wrist      ####
    # left elbow
    # left wrist
    ##############################
    try:
            tt = tfBuffer.lookup_transform('map', 'head_rgbd_sensor_link', rospy.Time())
                        
            trans,rot=read_tf(tt)
            #print ("############head",trans,rot)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print ( 'No head TF FOUND')

    found_joints={}
    for i, name in enumerate(['right_elbow','right_wrist','left_elbow','left_wrist']):
        if np.sum(np.asarray(poses[i]))==0:print(f'no {name} points')
        else:           
            print (poses[i],f' {name} pose wrt head')
            found_joints[name]=poses[i]
            
    print("FOUND JOINTS: ",found_joints)
    vd=[0,0,0]
    if 'right_wrist' in found_joints.keys() and 'right_elbow' in found_joints.keys():
        
        pc_np_array = np.array([(found_joints['right_elbow'][0], found_joints['right_elbow'][1], found_joints['right_elbow'][2]),
                                 (found_joints['right_wrist'][0],found_joints['right_wrist'][1],found_joints['right_wrist'][2])]
         , dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4')])
        #pc_np_array = np.array([found_joints['right_elbow'], found_joints['right_wrist'], (0.0, 0.0, 0.0)], dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4')])
        points_msg=ros_numpy.msgify(PointCloud2,pc_np_array,rospy.Time.now(),'head_rgbd_sensor_rgb_frame')
        cloud_out = do_transform_cloud(points_msg, tt)
        np_corrected=ros_numpy.numpify(cloud_out)
        corrected=np_corrected.reshape(pc_np_array.shape)
        print(corrected,'########################### Left', found_joints)
        #t=write_tf((corrected['x'][0],corrected['y'][0],corrected['z'][0]),(0,0,0,1),'right_elbow')
        #print (t)
        #b_st.sendTransform(t)
        #t=write_tf((corrected['x'][1],corrected['y'][1],corrected['z'][1]),(0,0,0,1),'right_wrist')
        #print (t)
        #b_st.sendTransform(t)
        elbow_xyz,wrist_xyz=[corrected['x'][0],corrected['y'][0],corrected['z'][0]],[corrected['x'][1],corrected['y'][1],corrected['z'][1]]
        v= np.asarray(wrist_xyz)-np.asarray(elbow_xyz)
        #print("ELBOW RIGHT",elbow_xyz)
        #print("WRIST RIGHT",wrist_xyz)
        vd = [-(wrist_xyz[0] - elbow_xyz[0]), -(wrist_xyz[1]-elbow_xyz[1]),-1-(wrist_xyz[2]-elbow_xyz[2])]
        
        vectD = [wrist_xyz[0]-elbow_xyz[0],wrist_xyz[1]-elbow_xyz[1],wrist_xyz[2]-elbow_xyz[2]]
        alfa = -wrist_xyz[2]/vectD[2]
        y=wrist_xyz[1]+alfa*vectD[1]
        x=wrist_xyz[0]+alfa*vectD[0]
        
        #t= elbow_xyz[2]-   v[2]
        #x= elbow_xyz[0]+ t*v[0]
        #y= elbow_xyz[1]+ t*v[1]
        print(x,y,'x,y')
        t=write_tf((x,y,0),(0,0,0,1),'pointing_right')
        b_st.sendTransform(t)
        res.x_r=x
        res.y_r=y
        res.z_r=0
        #
    else:
        res.x_r=0.0
        res.y_r=0.0
        res.z_r=0.0
        t=write_tf((0,0,0),(0,0,0,1),'pointing_right')
        b_st.sendTransform(t)

    vi=[0,0,0]
    if 'left_wrist'  in found_joints.keys() and 'left_elbow'  in found_joints.keys():
        pc_np_array = np.array([(found_joints['left_elbow'][0], found_joints['left_elbow'][1], found_joints['left_elbow'][2]),(found_joints['left_wrist'][0],found_joints['left_wrist'][1],found_joints['left_wrist'][2])]
         , dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4')])
        #pc_np_array = np.array([found_joints['right_elbow'], found_joints['right_wrist'], (0.0, 0.0, 0.0)], dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4')])
        points_msg=ros_numpy.msgify(PointCloud2,pc_np_array,rospy.Time.now(),'head_rgbd_sensor_rgb_frame')
        cloud_out = do_transform_cloud(points_msg, tt)
        np_corrected=ros_numpy.numpify(cloud_out)
        corrected=np_corrected.reshape(pc_np_array.shape)
        print(corrected,'###########################Right', found_joints)
        #t=write_tf((corrected['x'][0],corrected['y'][0],corrected['z'][0]),(0,0,0,1),'right_elbow')
        #print (t)
        #b_st.sendTransform(t)
        #t=write_tf((corrected['x'][1],corrected['y'][1],corrected['z'][1]),(0,0,0,1),'right_wrist')
        #print (t)
        #b_st.sendTransform(t)
        elbow_xyz,wrist_xyz=[corrected['x'][0],corrected['y'][0],corrected['z'][0]],[corrected['x'][1],corrected['y'][1],corrected['z'][1]]
        #print("ELBOW LEFT",elbow_xyz)
        #print("WRIST LEFT",wrist_xyz)
        v= np.asarray(wrist_xyz)-np.asarray(elbow_xyz)
        vi = [-(wrist_xyz[0] - elbow_xyz[0]), -(wrist_xyz[1]-elbow_xyz[1]),-1-(wrist_xyz[2]-elbow_xyz[2])]
        vectD = [wrist_xyz[0]-elbow_xyz[0],wrist_xyz[1]-elbow_xyz[1],wrist_xyz[2]-elbow_xyz[2]]
        alfa = -wrist_xyz[2]/vectD[2]
        y=wrist_xyz[1]+alfa*vectD[1]
        x=wrist_xyz[0]+alfa*vectD[0]
        
        #t= elbow_xyz[2]-   v[2]
        #x= elbow_xyz[0]+ t*v[0]
        #y= elbow_xyz[1]+ t*v[1]
        print(x,y, v,'x,y')
        t=write_tf((x,y,0),(0,0,0,1),'pointing_left')
        b_st.sendTransform(t)
        res.x_l=x
        res.y_l=y
        res.z_l=0
        
    else:

        res.x_l=0.0
        res.y_l=0.0
        res.z_l=0.0
        t=write_tf((0,0,0),(0,0,0,1),'pointing_left')
        b_st.sendTransform(t)

    
    if np.linalg.norm(vd)>np.linalg.norm(vi):
        print("Mano DERECHA levantada")
        res.x_l = -1.0
        res.y_l = -1.0
        res.z_l = -1.0

    else:
        print("Mano IZQUIERDA levantada")
        res.x_r = -1.0
        res.y_r = -1.0
        res.z_r = -1.0
        
    
    return res
    #print (poses[0])
    #if np.sum(np.asarray(poses[0]))==0:print('no r.e.')
    #else:
    #    t=write_tf(poses[0],(0,0,0,1),'right_elbow','head_rgbd_sensor_rgb_frame') 
    #    b_tf.sendTransform(t)
    #    rospy.sleep(0.25)
    #    tt=tfBuffer.lookup_transform('map','right_elbow',rospy.Time(0))
    #    pose,quat= read_tf(tt)
    #    t=write_tf(pose,(0,0,0,1),'right_elbow')
    #    b_st.sendTransform(t)
    
    """rospy.sleep(0.2)
                pose,quat= read_tf(tt)
                t=write_tf(pose,(0,0,0,1),'right_elbow')
                b_st.sendTransform(t)
                rospy.sleep(0.2)
                t=write_tf(poses[1],(0,0,0,1),'right_wrist','head_rgbd_sensor_rgb_frame') 
                b_tf.sendTransform(t)
                rospy.sleep(0.2)
                tt=tfBuffer.lookup_transform('map','right_wrist',rospy.Time(0))
                rospy.sleep(0.2)
                pose,quat= read_tf(tt)
                t=write_tf(pose,(0,0,0,1),'right_wrist')
                b_st.sendTransform(t)
            
                t=write_tf(poses[2],(0,0,0,1),'left_elbow','head_rgbd_sensor_rgb_frame') 
                b_tf.sendTransform(t)
                rospy.sleep(0.2)
                tt=tfBuffer.lookup_transform('map','left_elbow',rospy.Time(0))
                rospy.sleep(0.2)
                pose,quat= read_tf(tt)
                t=write_tf(pose,(0,0,0,1),'left_elbow')
                b_st.sendTransform(t)
            
                t=write_tf(poses[3],(0,0,0,1),'left_wrist','head_rgbd_sensor_rgb_frame') 
                b_tf.sendTransform(t)
                rospy.sleep(0.2)
                tt=tfBuffer.lookup_transform('map','left_wrist',rospy.Time(0))
                rospy.sleep(0.2)
                pose,quat= read_tf(tt)
                t=write_tf(pose,(0,0,0,1),'left_wrist')
                b_st.sendTransform(t)
            
            
            
            
            
                tt=tfBuffer.lookup_transform('map','right_wrist',rospy.Time(0))
                wrist_xyz,_= read_tf(tt)
                rospy.sleep(0.2)
                tt=tfBuffer.lookup_transform('map','right_elbow',rospy.Time(0))
                rospy.sleep(0.2)
                elbow_xyz,_= read_tf(tt)
                
                v= np.asarray(wrist_xyz)-np.asarray(elbow_xyz)
                print (v,elbow_xyz)
                t= elbow_xyz[2]-   v[2]
                x= elbow_xyz[0]+ t*v[0]
                y= elbow_xyz[1]+ t*v[1]
                t=write_tf((x,y,0),(0,0,0,1),'pointing_right')
                b_st.sendTransform(t)
                res.x_r=x
                res.y_r=y
                res.z_r=0
            
                tt=tfBuffer.lookup_transform('map','left_wrist',rospy.Time(0))
                wrist_xyz,_= read_tf(tt)
                rospy.sleep(0.2)
                tt=tfBuffer.lookup_transform('map','left_elbow',rospy.Time(0))
                rospy.sleep(0.2)
                elbow_xyz,_= read_tf(tt)
                
                v= np.asarray(wrist_xyz)-np.asarray(elbow_xyz)
                print (v,elbow_xyz)
                t= elbow_xyz[2]-   v[2]
                x= elbow_xyz[0]+ t*v[0]
                y= elbow_xyz[1]+ t*v[1]
                t=write_tf((x,y,0),(0,0,0,1),'pointing_left')
                b_st.sendTransform(t)
                res.x_l=x
                res.y_l=y
                res.z_l=0"""



    print (pose,quat)
    #tf_man.pub_static_tf(pos=poses[0],point_name='right_elbow', ref='head_rgbd_sensor_rgb_frame')
    #tf_man.change_ref_frame_tf(point_name='right_elbow')
    #tf_man.pub_static_tf(pos=poses[1],point_name='right_wrist', ref='head_rgbd_sensor_rgb_frame')
    #tf_man.change_ref_frame_tf(point_name='right_wrist')
    #tf_man.pub_static_tf(pos=poses[2],point_name='left_elbow', ref='head_rgbd_sensor_rgb_frame')
    #tf_man.change_ref_frame_tf(point_name='left_elbow')
    #tf_man.pub_static_tf(pos=poses[3],point_name='left_wrist', ref='head_rgbd_sensor_rgb_frame')
    #tf_man.change_ref_frame_tf(point_name='left_wrist')
    #wrist_xyz=tf_man.getTF(target_frame='right_wrist')
    #elbow_xyz=tf_man.getTF(target_frame='right_elbow')
    #res.x_r= x
    #res.y_r= y
    #res.z_r= 0
    #wrist_xyz=tf_man.getTF(target_frame='left_wrist')
    #elbow_xyz=tf_man.getTF(target_frame='left_elbow')
    #v= np.asarray(wrist_xyz[0])-np.asarray(elbow_xyz[0])
    #t=elbow_xyz[0][2]-v[2]
    #x= elbow_xyz[0][0]+ t*v[0]
    #y= elbow_xyz[0][1]+ t*v[1]
    #res.x_l= x
    #res.y_l= y
    #res.z_l= 0
    #tf_man.pub_static_tf(pos=[x,y,0],point_name='point_left')

    return res    

def detect_pointing2(points_msg):
    #tf_man = TF_MANAGER()
    res=Point_detectorResponse()
    points_data = ros_numpy.numpify(points_msg)    
    image_data = points_data['rgb'].view((np.uint8, 4))[..., [2, 1, 0]]   
    image=cv2.cvtColor(image_data, cv2.COLOR_BGR2RGB)
    pts= points_data
    print (image.shape)
    frame=image
    inHeight = frame.shape[0]
    inWidth = frame.shape[1]
    # Prepare the frame to be fed to the network
    inpBlob = cv2.dnn.blobFromImage(image, 1.0 / 255, (inWidth, inHeight), (0, 0, 0), swapRB=False, crop=False)
    # Set the prepared object as the input blob of the network
    net.setInput(inpBlob)
    output = net.forward()

    poses = getconectionJoints(output,inHeight,inWidth)
    imageDraw = drawSkeletons(image,poses,plot=False)
    print(os.path.expanduser( '~' )+"/Documents/tmpPOINTING.jpg")
    cv2.imwrite(os.path.expanduser( '~' )+"/Documents/tmpPOINTING.jpg",imageDraw)
    res.debug_image.append(bridge.cv2_to_imgmsg(imageDraw))

    # FALTA COMPARAR CON LA TF HUMAN Y VER CUAL ES LA MAS CERCANA (TODO LO DE ROS)
    dists=[]
    for i,pose in enumerate(poses):
        if pose[0,0] != 0:
            print(pose[0,0],pose[0,1],pose[0])
            pose_xyz =[points_data['x'][int(pose[0,1]), int(pose[0,0])],
                       points_data['y'][int(pose[0,1]), int(pose[0,0])],
                       points_data['z'][int(pose[0,1]), int(pose[0,0])]]
            dists.append(np.linalg.norm(pose_xyz)) 
            t=write_tf((pose_xyz[0],pose_xyz[1],pose_xyz[2]),(0,0,0,1),'person_'+str(i),parent_frame='head_rgbd_sensor_rgb_frame')
            b_st.sendTransform(t)
            rospy.sleep(0.3)
        elif pose[0,0] == 0 and pose[1,0] != 0:
            print(pose[1,0],pose[1,1])
            pose_xyz =[points_data['x'][int(pose[1,1]), int(pose[1,0])],
                       points_data['y'][int(pose[1,1]), int(pose[1,0])],
                       points_data['z'][int(pose[1,1]), int(pose[1,0])]]
            
            dists.append(np.linalg.norm(pose_xyz)) 
            t=write_tf((pose_xyz[0],pose_xyz[1],pose_xyz[2]),(0,0,0,1),'person_'+str(i),parent_frame='head_rgbd_sensor_rgb_frame')
            b_st.sendTransform(t)
            rospy.sleep(0.3)
        else:
            print("NO HAY DATOS PARA PUBLICAR")   


    print(np.min(dists),np.argmin(dists))
    k=0
    if len(dists)>1:
        # DE TODAS LAS DISTANCIAS OBTENGO EL INDICE DE LA MAS PEQUEÑA
        k= np.argmin(dists)
    # PUBLICO CODOS Y MANOS DE LA PERSONA k Y OBTENGO COORDENADAS RESPECTO A MAPA    
    codoD =[points_data['x'][int(poses[k,3,1]), int(poses[k,3,0])],
            points_data['y'][int(poses[k,3,1]), int(poses[k,3,0])],
            points_data['z'][int(poses[k,3,1]), int(poses[k,3,0])]]
    codoI =[points_data['x'][int(poses[k,6,1]), int(poses[k,6,0])],
            points_data['y'][int(poses[k,6,1]), int(poses[k,6,0])],
            points_data['z'][int(poses[k,6,1]), int(poses[k,6,0])]]
    manoD =[points_data['x'][int(poses[k,4,1]), int(poses[k,4,0])],
            points_data['y'][int(poses[k,4,1]), int(poses[k,4,0])],
            points_data['z'][int(poses[k,4,1]), int(poses[k,4,0])]]
    manoI =[points_data['x'][int(poses[k,7,1]), int(poses[k,7,0])],
            points_data['y'][int(poses[k,7,1]), int(poses[k,7,0])],
            points_data['z'][int(poses[k,7,1]), int(poses[k,7,0])]]
            
    
    t=write_tf((codoD[0],codoD[1],codoD[2]),(0,0,0,1),'codoD',parent_frame='head_rgbd_sensor_rgb_frame')
    b_st.sendTransform(t)
    rospy.sleep(0.3)
    t=write_tf((codoI[0],codoI[1],codoI[2]),(0,0,0,1),'codoI',parent_frame='head_rgbd_sensor_rgb_frame')
    b_st.sendTransform(t)
    rospy.sleep(0.3)
    t=write_tf((manoD[0],manoD[1],manoD[2]),(0,0,0,1),'manoD',parent_frame='head_rgbd_sensor_rgb_frame')
    b_st.sendTransform(t)
    rospy.sleep(0.3)
    t=write_tf((manoI[0],manoI[1],manoI[2]),(0,0,0,1),'manoI',parent_frame='head_rgbd_sensor_rgb_frame')
    b_st.sendTransform(t)
    rospy.sleep(0.7)

    change_ref_frame_tf(point_name='codoD')
    change_ref_frame_tf(point_name='codoI')
    change_ref_frame_tf(point_name='manoD')
    change_ref_frame_tf(point_name='manoI')
    
    codoD, _ =getTF(target_frame='codoD')
    codoI, _ =getTF(target_frame='codoI')
    manoD, _ =getTF(target_frame='manoD')
    manoI, _ =getTF(target_frame='manoI')

    ds=[manoD[2]-codoD[2],manoI[2]-codoI[2]]
   
    v1=[-(manoD[0]-codoD[0]),-(manoD[1]-codoD[1]),ds[np.argmin(ds)]-(manoD[2]-codoD[2])]
    v2=[-(manoI[0]-codoI[0]),-(manoI[1]-codoI[1]),ds[np.argmin(ds)]-(manoI[2]-codoI[2])]
    
    vectD = [manoD[0]-codoD[0],manoD[1]-codoD[1],manoD[2]-codoD[2]]
    alfa = -manoD[2]/vectD[2]
    y=manoD[1]+alfa*vectD[1]
    x=manoD[0]+alfa*vectD[0]
    #print(x,y,'x,y DER')
    t=write_tf((x,y,0),(0,0,0,1),'pointing_right')
    b_st.sendTransform(t)
    res.x_r=x
    res.y_r=y
    res.z_r=0

    rospy.sleep(0.3)

    vectD = [manoI[0]-codoI[0],manoI[1]-codoI[1],manoI[2]-codoI[2]]
    alfa = -manoI[2]/vectD[2]
    y=manoD[1]+alfa*vectD[1]
    x=manoD[0]+alfa*vectD[0]
    #print(x,y,'x,y IZQ')
    t=write_tf((x,y,0),(0,0,0,1),'pointing_left')
    b_st.sendTransform(t)
    res.x_l=x
    res.y_l=y
    res.z_l=0
    
    if np.linalg.norm(v1) > np.linalg.norm(v1):
        print("Mano DERECHA levantada")
        res.x_l = -1.0
        res.y_l = -1.0
        res.z_l = -1.0
        #
    else:
        print("Mano IZQUIERDA levantada")
        res.x_r = -1.0
        res.y_r = -1.0
        res.z_r = -1.0
    return res
    

#><>>>>>>>>>>>>>><<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


    i = 0 #Face
    #i = 1# Neck
    probMap = output[0, i, :, :]
    probMap = cv2.resize(probMap, (inWidth, inHeight))
    cent= probmap_to_3d_mean(points_data,probMap)
    print (cent)
    if np.isnan(cent.any()):return Human_detectorResponse()
    print (cent)
    if np.isnan(cent.any()):cent=np.zeros(3)
    res=Human_detectorResponse()
    res.x= cent[0]
    res.y= cent[1]
    res.z= cent[2]
    
    return res    


def get_points(frame,inHeight,inWidth,output,threshold=0.1):
    
    h=output.shape[2]
    w=output.shape[3]
    points=[]

    for i in range(25):
        probMap = output[0,i,:,:]
        minVal,prob,minLoc,point = cv2.minMaxLoc(probMap)
        print("P: ",point)
        x = (inWidth * point[0]) / w
        y = (inHeight * point[1]) / h

        if prob > threshold : 
            cv2.circle(frame,(int(x),int(y)),5,(0,255,255),-1)
            cv2.putText(frame,str(i),(int(x),int(y)),
                        cv2.FONT_HERSHEY_SIMPLEX,0.6,(255,165,5),1,
                        lineType=cv2.LINE_AA)

            points.append((x,y))
        else:
            points.append(None)

    return points


# IN PROGRESS
def detect_all(points_msg):
    direct=os.path.expanduser( '~' )
    im=cv2.imread(direct+"/Documents/Tests/persons2.jpg")
    print(im.shape)
    points_data = ros_numpy.numpify(points_msg)    
    image_data = points_data['rgb'].view((np.uint8, 4))[..., [2, 1, 0]]   
    image=cv2.cvtColor(image_data, cv2.COLOR_BGR2RGB)
    #image = points_data['rgb'].view((np.uint8, 4))[..., [2, 1, 0]]
    #rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    print (image.shape)
    frame=im
    inHeight = frame.shape[0]
    inWidth = frame.shape[1]

    keypoints=[]
    # Prepare the frame to be fed to the network
    inpBlob = cv2.dnn.blobFromImage(frame, 1.0 / 255, (inWidth, inHeight), (0, 0, 0), swapRB=False, crop=False)

    # Set the prepared object as the input blob of the network
    net.setInput(inpBlob)

    output = net.forward()

    points = get_points(frame,inHeight,inWidth,output)
    print("POINTSSSS",len(points),points)
    cv2.imshow("DRAW",frame)
    cv2.waitKey(0)

    """
    i = 0 #Face
    #i = 1# Neck
    print("SHAPEs",output.shape)
    probMap = output[0, i, :, :]
    print(probMap.shape)
    probMap = cv2.resize(probMap, (inWidth, inHeight))
    minVal, prob, minLoc, point = cv2.minMaxLoc(probMap)
 
        #imr=cv2.bitwise_or(im, probMap)
    cv2.imshow("A",cv2.cvtColor(im, cv2.COLOR_BGR2RGB))
    for i in range(output.shape[1]):
        probM=output[0,i,:,:]
        probM=cv2.resize(probM, (inWidth, inHeight))

        cv2.imshow("B",probM)
        cv2.waitKey(0)

    """
    cv2.destroyAllWindows()

    return Point_detectorResponse() 


# FUNCIONES PARA DETECTAR TODOS LOS KEYPOINTS

#--------------------------------------
def getKeypoints(output,inWidth, inHeight,numKeys=9,thresholdKey=110):
    # se obtiene primero los keypoints 
    keypoints=[]
    for i in range (numKeys):
        probMap = output[0, i, :, :]
        probMap = cv2.resize(probMap, (inWidth, inHeight))
        mapSmooth = cv2.GaussianBlur(probMap,(3,3),0,0)
        _,mapMask = cv2.threshold(mapSmooth*256,thresholdKey,255,cv2.THRESH_BINARY)
        contours,_ = cv2.findContours(np.uint8(mapMask), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            blobMask = np.zeros(mapMask.shape)
            blobMask = cv2.fillConvexPoly(blobMask, cnt, 1)
            maskedProbMap = mapSmooth * blobMask
            _, maxVal, _, maxLoc = cv2.minMaxLoc(maskedProbMap)
            keypoints.append(maxLoc + (probMap[maxLoc[1], maxLoc[0]],) +(i,))
    return keypoints


#--------------------------------------
def getDistances(keysPart,gP):
    matchesInMask=[]
    for key in keysPart :

        tmp=[]
        for j in range(len(gP)):

            if gP[j] and [key[0],key[1]] in gP[j]:
                tmp=[]
                tmp.append([[key[0],key[1]],key[-1],j,-1,-1])
                break

            elif gP[j] and not [key[0],key[1]] in gP[j]:
                restas=np.subtract(gP[j],[key[0],key[1]])
                distMIN = np.linalg.norm(restas,axis=1)
                tmp.append([[key[0],key[1]],key[-1],j,np.argmin(distMIN),np.min(distMIN)])
        matchesInMask.append(tmp)
    return matchesInMask

#--------------------------------------
def ordenaKeypoints(keypoints,numKeys):
    # ORDENO DE IZQUIERDA A DERECHA (LA PRIMER COORDENADA)
    l=0
    for i in range(numKeys):
        tmp = [item[0] for item in [item for item in keypoints if item[-1] == i]]
        order = np.argsort(tmp) if i==0 else np.concatenate((order,np.argsort(tmp)+l),axis=None)
        l+=len(tmp)
    
    return [keypoints[i] for i in order]


#--------------------------------------
def asignaEspacios(lista,maxPeople):
    espacios = list(range(maxPeople))

    defaul=[]
    for keyT in lista:
        if len(keyT)==1:
            defaul.append(keyT[0][2])
    #descarto espacios ya ocupados
    for d in defaul:
        espacios.remove(d)
    #lista con valores ya asignados
    subLista=[x[0] for x in lista if len(x)==1]

    while espacios:
        #lista con valores por asignar, guardo indice para despues modificar con asignacion
        sublista2=[x for x in lista if len(x)>1]
        indexes=[i for i,x in enumerate(lista) if len(x)>1]
        tmp=[]
        if len(sublista2) == 1:
            lista[indexes[0]] = [[sublista2[0][0][0],sublista2[0][0][1],espacios[0],-1,-1]]
            break
        for x in sublista2:
            for key in x:
                if key[2] == espacios[0]:
                    tmp.append(key)
        # esta lista es la nueva de acuerdo a la asignacion de la distancia mas corta
        keyTmp=[tmp[np.argmin([x[-1] for x in tmp])][0],
                tmp[np.argmin([x[-1] for x in tmp])][1],
               tmp[np.argmin([x[-1] for x in tmp])][2],
                -1,
                -1]
        subLista.append(keyTmp)
        lista[indexes[np.argmin([x[-1] for x in tmp])]] = [keyTmp]
        espacios.pop(0)

    #quito [] sobrantes
    lista = [x[0] for x in lista]
    return lista


#----------------------
# FUNCION PRINCIPAL
def getconectionJoints(output,inHeight,inWidth,thresKeyP=110,numKeys=9 ):
    # relacion entre indice de heatmaps y joints -> [hm,jointA,jointB]
    conections = [[57,0,1],[40,1,2],[43,2,3],[45,3,4],[48,1,5],[51,5,6],[53,6,7]] #primero puede ser NO necesario
    
    # se obtiene primero los keypoints 
    keypoints = getKeypoints(output,inWidth, inHeight,numKeys,thresholdKey=thresKeyP)

    # cuento max personas encontradas
    conteo=Counter([i[-1] for i in keypoints])
    maxPeople = max(list(conteo.values()))
    avgPeople = round(sum(list(conteo.values()))/len(list(conteo.values())))
    if maxPeople > avgPeople :
        maxPeople = avgPeople
    sk = np.zeros([maxPeople,8,2])


    # por cada conexion se compara los keypoints # AQUI EMPEZARIA EL LOOP POR CADA CONEXION
    for con in conections:
        #con = conections[0]    # OPCION PARA 1 CASO, DESCOMENTAR Y COMENTAR LINEA ANTERIOR (QUITAR IDENTACION)
        gP=[]
        for i in range(maxPeople):
            gP.append([])
        #
        print("CONECTION",con)      
        probMap = output[0, con[0], :, :]
        probMap = cv2.resize(probMap, (inWidth, inHeight))
        cv2.imwrite(os.path.expanduser( '~' )+"/Documents/heatmap_"+str(con[0])+".jpg",probMap)
        mapSmooth = cv2.GaussianBlur(probMap,(3,3),0,0)
        thresh =-256 if mapSmooth.max() < 0.1 else 256
        minthresh = mapSmooth.max()*thresh/2 if thresh == 256 else mapSmooth.min()*thresh/2
        if minthresh >15:
            _,mapMask = cv2.threshold(mapSmooth*thresh,minthresh-10,255,cv2.THRESH_BINARY)
        else:
            _,mapMask = cv2.threshold(mapSmooth*thresh,minthresh-1,255,cv2.THRESH_BINARY)
        contours,_ = cv2.findContours(np.uint8(mapMask), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # OBTENGO CENTROIDES PARA PODER SEPARAR LAS MASCARAS EN SUBCONJUNTOS DE COORDENADAS
        cents=[]
        gruposDist=False
        for cont in contours:
            cents.append(np.mean(cont, axis=0)[0])
        #ordeno de izquierda a derecha (en eje x)
        cents = [cents[i] for i in np.argsort([item[0] for item in cents])]


        gruposDist=len(cents)< maxPeople
        #obtengo coordenadas de todos los puntos que no tengan valor en cero
        maskV = np.transpose(np.nonzero(mapMask == 255))

        # SEPARO POR CONJUNTOS DE ACUERDO A LOS CENTROIDES MAS CERCANOS
        for v in maskV:
            distMin=np.linalg.norm(np.subtract(cents,[v[1],v[0]]),axis=1)
            # agrego a la lista en el grupo de la menor distancia
            gP[np.argmin(distMin)].append([v[1],v[0]]) # primero [1,0] para 'acomodar'

        keyspartA = [k for k in keypoints if k[-1]==con[1] ]
        keyspartB = [k for k in keypoints if k[-1]==con[2] ]

        if keyspartA[0][-1] == 1 and gruposDist and sk[0,0,0]==0:
            # CASO NO IDEAL QUE NO PUEDE CREAR 'BIEN' DESDE UN INICIO
            return []

        # CASO INICIAL, SI NO HUBO MISMO NUMERO DE PERSONAS y CABEZAS/CUELLOS DETECTADAS CON REGIONES
        if keyspartA[0][-1] == 0 and gruposDist and len(keyspartA) == maxPeople:
            # CASO EN QUE SE PUEDE AGREGAR CABEZAS AUNQUE HAYA MENOS REGIONES, SOLO SI HAY MISMO
            # NUMERO DE CABEZAS QUE PERSONAS
            for i in range(len(keyspartA)):
                sk[i,0,0] = keyspartA[i][0]
                sk[i,0,1] = keyspartA[i][1] 
        if keyspartA[0][-1] == 0 and gruposDist and len(keyspartA) != maxPeople:
            print("CASO PENDIENTE de TRATAR o quitar")

        # ELSE HACER LO DE ABAJO CON A y despues con B
        else:

            # AQUI OBTENGO DISTANCIAS MINIMAS O PERTENENCIAS DE LOS PUNTOS (JOINTS) A UN GRUPO 
            # matchesInMask_ = [ (key_x,key_y), num_joint, grupo_encontrado, index_distMin, distMin ]
            matchesInMaskA=getDistances(keyspartA,gP)
            matchesInMaskB=getDistances(keyspartB,gP)

            modifA = False
            # CASOS INICIALES E IDEALES
            if max([len(i) for i in matchesInMaskA]) == 1 and (matchesInMaskA[0][0][1]==0 or matchesInMaskA[0][0][1]==1):
                for item in matchesInMaskA:
                    sk[item[0][2],item[0][1],0] = item[0][0][0]
                    sk[item[0][2],item[0][1],1] = item[0][0][1]
            else:
                # CASO NO IDEAL DEL PRIMER GRUPO, AL MENOS UN KEYPOINT FUERA DE UNA REGION
                listaCorrespA = asignaEspacios(matchesInMaskA,maxPeople)
                modifA = True
            # CASO IDEAL PARA EL SEGUNDO GRUPO DE KEYPOINTS (B), TODOS DENTRO DE UNA REGION DE CORRESPONDENCIA
            if max([len(i) for i in matchesInMaskB]) == 1:
                for item in matchesInMaskB:
                    sk[item[0][2],item[0][1],0] = item[0][0][0]
                    sk[item[0][2],item[0][1],1] = item[0][0][1]
            
            #  CASO EN QUE HUBO KEYPOINTS FUERA DE REGIONES DE CORRESPONDENCIA
            else:
                # SE ASIGNA REGIONES DE ACUERDO A LAS DISTANCIAS MINIMAS DE CADA KEYPOINT A LA(s) REGION(es)
                listaCorrespB = asignaEspacios(matchesInMaskB,maxPeople)
                # CASO EN QUE NO HUBO MENOR NUMERO DE REGIONES 
                if not gruposDist:
                    for item in listaCorrespB:
                        sk[item[2],item[1],0] = item[0][0]
                        sk[item[2],item[1],1] = item[0][1]
                        
                # CASO EN QUE SI HUBO MENOR NUMERO DE REGIONES 
                # SE COMPARA CON DATOS ANTERIORES QUE ESTARAN 'CORRECTAS'
                else:
                    # EN CASO QUE HUBO MODIFICACION AL PRIMER GRUPO (A)
                    if modifA:
                        for itemB in listaCorrespB:
                            # BUSCO CORRESPONDENCIA PARA CADA PUNTO B CON EL A
                            tmp = [x for x in listaCorrespA if x[2]==itemB[2]][0]
                            # COMO A DEL PASO ANTERIOR ES CORRECTO, BUSCO EN QUE GRUPO(REGION) SE ASIGNO 
                            indexCorrecto = [i for i,item in enumerate(sk) if tmp[0] in item]
                            # REASIGNO B AL GRUPO DE A DEL PASO ANTERIOR
                            sk[indexCorrecto[0],itemB[1],0] = itemB[0][0]
                            sk[indexCorrecto[0],itemB[1],1] = itemB[0][1]
                    
                    # EN CASO QUE NO HUBO MODIFICACIONES AL PRIMER GRUPO (A)
                    else:
                        for itemB in listaCorrespB:
                            # BUSCO CORRESPONDENCIA PARA CADA PUNTO B CON EL A
                            tmp = [x for x in matchesInMaskA if x[0][2]==itemB[2]]
                            # COMO A DEL PASO ANTERIOR ES CORRECTO, BUSCO EN QUE GRUPO SE ASIGNO 
                            indexCorrecto = [i for i,item in enumerate(sk) if tmp[0] in item]
                            # REASIGNO B AL GRUPO DE A DEL PASO ANTERIOR
                            sk[indexCorrecto[0],itemB[1],0] = itemB[0][0]
                            sk[indexCorrecto[0],itemB[1],1] = itemB[0][1]

    return sk

#----------------------
def drawSkeletons(frame,sk,plot=False):
    colors=[(41,23,255),(99,1,249),(251,10,255),(10,75,255),(41,243,186),(10,10,255),
                    (25,136,253),(40,203,253),(0,218,143),(0,218,116),(78,218,0),(253,183,31),
                    (148,241,4),(239,255,1),(253,145,31),(253,80,31),(248,8,207),(248,8,76)]
    conections = [[57,0,1],[40,1,2],[43,2,3],[45,3,4],[48,1,5],[51,5,6],[53,6,7]] #primero puede ser NO necesario
    
    rgbbkg = np.copy(frame)
    for s in sk:
        rgbbkg = cv2.circle(rgbbkg,(int(s[0][0]),int(s[0][1])),6,(255,255,0),-1)
        rgbbkg = cv2.circle(rgbbkg,(int(s[1][0]),int(s[1][1])),6,(255,255,0),-1)
        rgbbkg = cv2.line(rgbbkg,
                              (int(s[conections[0][1],0]),int(s[conections[0][1],1])),
                              (int(s[conections[0][2],0]),int(s[conections[0][2],1])),
                              colors[0],
                              2)
    for i in range(1,len(conections)):
        for s in sk:
            if int(s[conections[i][2],0]) != 0:
                rgbbkg = cv2.circle(rgbbkg,(int(s[conections[i][2],0]),int(s[conections[i][2],1])),6,(255,255,0),-1)
            if int(s[conections[i][1],0]) != 0 and int(s[conections[i][2],0]) != 0:
                rgbbkg = cv2.line(rgbbkg,
                              (int(s[conections[i][1],0]),int(s[conections[i][1],1])),
                              (int(s[conections[i][2],0]),int(s[conections[i][2],1])),
                              colors[i],
                              2)
    
    if plot:
        plt.imshow(rgbbkg)
    return rgbbkg

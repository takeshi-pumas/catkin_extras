#! /usr/bin/env python3
import tf
import cv2
import rospy  
import tf2_ros                                    
from segmentation.srv import Segmentation, SegmentationResponse 
from cv_bridge import CvBridge
from object_classification.srv import *
import tf2_ros    
from segmentation.msg import *

import numpy as np
import ros_numpy
import os
import matplotlib.pyplot as plt

from sensor_msgs.msg import Image , LaserScan , PointCloud2
from geometry_msgs.msg import TransformStamped, Pose
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from mpl_toolkits.axes_grid1 import make_axes_locatable
#-----------------------------------------------------------------
global tf_listener, ptcld_lis, broadcaster , bridge


rospy.init_node('plane_segmentation') 
tfBuffer = tf2_ros.Buffer()
tfBuffer = tf2_ros.Buffer()
listener2 = tf2_ros.TransformListener(tfBuffer)
listener = tf.TransformListener()

broadcaster = tf2_ros.TransformBroadcaster()
tf_static_broadcaster = tf2_ros.StaticTransformBroadcaster()

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
def correct_points(points_msg,low=0.27,high=1000):

    # Function that transforms Point Cloud reference frame from  head, to map. (i.e. sensor coords to map coords )
    # low  high params Choose corrected plane to segment  w.r.t. head link 
    # img= correct_points() (Returns rgbd depth corrected image)    

    #data = rospy.wait_for_message('/hsrb/head_rgbd_sensor/depth_registered/rectified_points', PointCloud2)
    np_data=ros_numpy.numpify(points_msg)

    try:
        trans = tfBuffer.lookup_transform('map', 'head_rgbd_sensor_link', rospy.Time())
                    
        trans,rot=read_tf(trans)
        print ("############head",trans,rot)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        print ( 'No  head TF FOUND')

    #trans,rot=tf_listener.lookupTransform('/map', '/head_rgbd_sensor_rgb_frame', rospy.Time(0))
    #print ("############TF1",trans,rot)


    eu=np.asarray(tf.transformations.euler_from_quaternion(rot))
    t=TransformStamped()
    rot=tf.transformations.quaternion_from_euler(eu[0],eu[1],eu[2])
    #rot=tf.transformations.quaternion_from_euler(-eu[1],0,0)
    t.header.stamp = points_msg.header.stamp
    
    t.transform.rotation.x = rot[0]
    t.transform.rotation.y = rot[1]
    t.transform.rotation.z = rot[2]
    t.transform.rotation.w = rot[3]

    cloud_out = do_transform_cloud(points_msg, t)
    np_corrected=ros_numpy.numpify(cloud_out)
    corrected=np_corrected.reshape(np_data.shape)

    img= np.copy(-corrected['z'])

    img[np.isnan(img)]=2

    img_corrected = np.where((img<trans[2]*0.96) ,img,5)
#    img_corrected = np.where((img<trans[2]+0.07)&(img>trans[2]-0.05) ,img,5)
    return img_corrected , corrected

#-----------------------------------------------------------------
"""
def plane_seg (points_msg,lower=500 ,higher=50000,reg_ly= 30,reg_hy=600,plt_images=False):
    
    points_data=ros_numpy.numpify(points_msg)
    image = points_data['rgb'].view((np.uint8, 4))[..., [2, 1, 0]]
    #hsv_image = cv2.cvtColor(image, cv2.COLOR_RGB2HSV_FULL)
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    img_corrected,pts_corrected= correct_points(points_msg)
    contours, hierarchy = cv2.findContours(img_corrected.astype('uint8'),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    i=0
    cents=[]
    points=[]
    images=[]
    for i, contour in enumerate(contours):

        area = cv2.contourArea(contour)
        if area > lower and area < higher :
            M = cv2.moments(contour)
            # calculate x,y coordinate of center
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])




            # calculate moments for each contour
            if (cY > reg_ly and cY < reg_hy  ):

                boundRect = cv2.boundingRect(contour)
                #just for drawing rect, dont waste too much time on this
                #image_aux= iimmg[boundRect[1]:boundRect[1]+max(boundRect[2],boundRect[3]),boundRect[0]:boundRect[0]+max(boundRect[2],boundRect[3])]
                

                image_aux= image[boundRect[1]:boundRect[1]+boundRect[3],boundRect[0]:boundRect[1]+boundRect[2]]
                images.append(image_aux)
                image_aux= img_corrected[boundRect[1]:boundRect[1]+boundRect[3],boundRect[0]:boundRect[0]+boundRect[2]]

                mask=np.where(image_aux!=5)
                npmask=np.asarray(mask).T
                hsv_image=cv2.rectangle(hsv_image,(boundRect[0], boundRect[1]),(boundRect[0]+boundRect[2], boundRect[1]+boundRect[3]), (0,0,0), 2)
                cv2.circle(hsv_image, (cX, cY), 5, (255, 255, 255), -1)
                cv2.putText(hsv_image, "centroid_"+str(i)+"_"+str(cX)+','+str(cY)    ,    (cX - 25, cY - 25)   ,cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 0), 2)
                print ('cX,cY',cX,cY,'len mask',len(npmask))
                xyz=[]
                if len (npmask)>0:
                    for a in npmask:
                        ix,iy=a[0],a[1]
                        aux=(np.asarray((points_data['x'][boundRect[1]+ix,boundRect[0]+iy],points_data['y'][boundRect[1]+ix,boundRect[0]+iy],points_data['z'][boundRect[1]+ix,boundRect[0]+iy])))
                        #print (aux)
                        if np.isnan(aux[0]) or np.isnan(aux[1]) or np.isnan(aux[2]):
                                'reject point'
                        else:
                            xyz.append(aux)
                
                xyz=np.asarray(xyz)
                #print (xyz)
                cent=xyz.mean(axis=0)
                cents.append(cent)
                print (cent)
                points.append(xyz)
                
            else:   
                print ('cent out of region... rejected')
    return(cents,np.asarray(points), images,hsv_image)
"""

#-----------------------------------------------------    
def plot_with_cbar(image,cmap="jet"):
    ax = plt.subplot()
    im=ax.imshow(image, cmap=cmap)
    divider = make_axes_locatable(ax)
    cax = divider.append_axes("right", size="5%", pad=0.05)
    plt.colorbar(im, cax=cax)
    plt.show()
#-----------------------------------------------------
def segment_table(points_data,zs_no_nans,obj_lMax=0.8):

    histogram, bin_edges =(np.histogram(zs_no_nans, bins=50))
    t = tfBuffer.lookup_transform('map', 'head_rgbd_sensor_link', rospy.Time())
    trans=t.transform.translation.z
    print(trans)
    img_corrected = np.where((-zs_no_nans < trans*0.999) ,zs_no_nans,1.0)  #FLOOR

    #print((trans*0.999)-0.72 +0.2)
    #Quita a una altura
    plane_height= (trans)+bin_edges[histogram[:-1].argmax()+1]
    img_corrected = np.where(    (-zs_no_nans <  (trans*0.999)-plane_height-0.01),zs_no_nans,1)

    # Quita objetos lejanos
    lenZ_no_nans=np.where(~np.isnan(points_data['z']),points_data['z'],-5)
    lenZ_corrected=np.where(lenZ_no_nans<obj_lMax,lenZ_no_nans,-5)

    # quita Z en los que los X sean -5
    for r in range(img_corrected.shape[0]):
        for c in range(img_corrected.shape[1]):
            if lenZ_corrected[r,c]<=-5:
                img_corrected[r,c]=1
    return img_corrected
#-----------------------------------------------------    
def segment_floor(points_data,zs_no_nans,obj_hMax=0.85,obj_lMax=1.5):
    # obj_hMax -> altura maxima para objetos para segmentar
    # obj_lMax -> distancia de objetos maxima para segmentar

    # Quita piso y mayor a una cierta altura
    t = tfBuffer.lookup_transform('map', 'head_rgbd_sensor_link', rospy.Time())
    trans=t.transform.translation.z
    img_corrected = np.where((zs_no_nans < -obj_hMax),zs_no_nans,1)
    img_corrected = np.where((img_corrected >-trans-0.15),img_corrected,1)


    # Quita objetos lejanos
    ls_no_nans=np.where(~np.isnan(points_data['z']),points_data['z'],-5)

    lZ_corrected=np.where(ls_no_nans>obj_lMax,ls_no_nans,-5)

    # Con esto, quita en z los que en X esten con -5
    for r in range(img_corrected.shape[0]):
        for c in range(img_corrected.shape[1]):
            if lZ_corrected[r,c]<=-5:
                img_corrected[r,c]=1

    return img_corrected

#-----------------------------------------------------------------    
    
def plane_seg2(points_msg,hg=0.85,lg=1.5,lower=100 ,higher=50000,reg_ly= 30,reg_hy=600,plot=False):
    points_data = ros_numpy.numpify(points_msg)    
    image_data = points_data['rgb'].view((np.uint8, 4))[..., [2, 1, 0]]   
    image=cv2.cvtColor(image_data, cv2.COLOR_BGR2RGB)
    image = points_data['rgb'].view((np.uint8, 4))[..., [2, 1, 0]]
    rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    print (image.shape)

    _,corrected=correct_points(points_msg)
    zs_no_nans=np.where(~np.isnan(corrected['z']),corrected['z'],1)
    histogram, bin_edges =(np.histogram(zs_no_nans, bins=100))
    t = tfBuffer.lookup_transform('map', 'head_rgbd_sensor_link', rospy.Time())
    trans=t.transform.translation.z
    plane_height= (trans)+bin_edges[histogram[:-1].argmax()+1]
    print(plane_height)
    if plane_height<0.1:
        print("Segmentacion en: Piso")
        im_corrected=segment_floor(points_data,zs_no_nans,obj_hMax=hg,obj_lMax=lg)
    else:
        print("Segmentacion en: Mesa")
        im_corrected=segment_table(points_data,zs_no_nans,obj_lMax=lg)
    
    if plot:
        cv2.imshow("Image to segment",im_corrected)
        cv2.waitKey(0)

        cv2.destroyAllWindows()
    contours, hierarchy = cv2.findContours(im_corrected.astype('uint8'),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    i=0
    cents=[]
    points=[]
    images=[]
    for i, contour in enumerate(contours):

        area = cv2.contourArea(contour)
        if area > lower and area < higher :
            M = cv2.moments(contour)
            # calculate x,y coordinate of center
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            if (cY > reg_ly and cY < reg_hy  ):

                boundRect = cv2.boundingRect(contour)
                #just for drawing rect, dont waste too much time on this
                #image_aux= iimmg[boundRect[1]:boundRect[1]+max(boundRect[2],boundRect[3]),boundRect[0]:boundRect[0]+max(boundRect[2],boundRect[3])]
                

                image_aux= image[boundRect[1]:boundRect[1]+boundRect[3],boundRect[0]:boundRect[1]+boundRect[2]]
                images.append(image_aux)
                image_aux= im_corrected[boundRect[1]:boundRect[1]+boundRect[3],boundRect[0]:boundRect[0]+boundRect[2]]

                mask=np.where(image_aux!=5)
                npmask=np.asarray(mask).T
                rgb_image=cv2.rectangle(rgb_image,(boundRect[0], boundRect[1]),(boundRect[0]+boundRect[2], boundRect[1]+boundRect[3]), (255,255,0), 2)
                cv2.circle(rgb_image, (cX, cY), 5, (255, 255, 255), -1)
                cv2.putText(rgb_image, "centroid_"+str(i)+"_"+str(cX)+','+str(cY)    ,    (cX - 25, cY - 25)   ,cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 0), 2)
                print ('cX,cY',cX,cY,'len mask',len(npmask))
                xyz=[]
                if len (npmask)>0:
                    for a in npmask:
                        ix,iy=a[0],a[1]
                        aux=(np.asarray((points_data['x'][boundRect[1]+ix,boundRect[0]+iy],points_data['y'][boundRect[1]+ix,boundRect[0]+iy],points_data['z'][boundRect[1]+ix,boundRect[0]+iy])))
                        #print (aux)
                        if np.isnan(aux[0]) or np.isnan(aux[1]) or np.isnan(aux[2]):
                                'reject point'
                        else:
                            xyz.append(aux)
                
                xyz=np.asarray(xyz)
                #print (xyz)
                cent=xyz.mean(axis=0)
                cents.append(cent)
                print (cent)
                points.append(xyz)
                
            else:   
                print ('cent out of region... rejected')
    return cents,np.asarray(points), images,rgb_image

#-----------------------------------------------------------------
         

    


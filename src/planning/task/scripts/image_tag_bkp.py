# -*- coding: utf-8 -*-


#!/usr/bin/env python
    
import numpy as np
import rospy
import ros_numpy
import tf2_ros
import tf
import os
import message_filters
from sensor_msgs.msg import Image , LaserScan , PointCloud2
from geometry_msgs.msg import TransformStamped
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

from smach_utils2 import *

from object_classification.srv import *
#from utils_srv import RGBD


from std_msgs.msg import String


def nothing(x):
    pass

##############################################################################################################################################################################################################################################################################################################################
def callback(points_msg):




    #print('got imgs msgs')
    points_data = ros_numpy.numpify(points_msg)    
    image_data = points_data['rgb'].view((np.uint8, 4))[..., [2, 1, 0]]   #JUST TO MANTAIN DISPLAY
    image=cv2.cvtColor(image_data, cv2.COLOR_BGR2RGB)




    img=rgbd.get_image()
    img=cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
   

    cv2.imshow('xtion rgb'	, image)

     

    cv2.imshow('class rgbd'  , img)
    cv2.createTrackbar('R', 'class rgbd', 0, 255, nothing)
    
    #print (r)

    # Process any keyboard commands
    keystroke = cv2.waitKey(1)
    
   
    
    

    
    if 32 <= keystroke and keystroke < 128:
        key = chr(keystroke).lower()
        print (key)
    #    
    #    
    #   
        
        if key=='u': 

            print('cha')
            ptcld_lis.unregister()
            
            



        if key=='s': 

            r = cv2.getTrackbarPos('R', 'class rgbd')
            print ('#################################',r)
            
            head.set_joint_values([ 0.1, -0.5])
            res=segmentation_server.call()
            print( 'segmenting')
            brazo.set_named_target('go')
            origin_map_img=[round(img_map.shape[0]*0.5) ,round(img_map.shape[1]*0.5)]   
            if len(res.poses.data)==0:print('no objs')
            else:
                poses=np.asarray(res.poses.data)
                poses=poses.reshape((int(len(poses)/3) ,3     )      )  
                num_objs=len(poses)
                print (num_objs)
                for i,pose in enumerate(poses):
                    #print (f'Occupancy map at point object {i}-> pixels ',origin_map_img[1]+ round(pose[1]/pix_per_m),origin_map_img[0]+ round(pose[0]/pix_per_m), img_map[origin_map_img[1]+ round(pose[1]/pix_per_m),origin_map_img[0]+ round(pose[0]/pix_per_m)])
                    point_name=f'object_{i}'
                    tf_man.pub_static_tf(pos=pose, point_name=point_name, ref='head_rgbd_sensor_rgb_frame')## which object to choose   #TODO
                    rospy.sleep(0.3)
                    tf_man.change_ref_frame_tf(point_name=point_name, new_frame='map')
                    rospy.sleep(0.3)
                    pose,_= tf_man.getTF(point_name)
                    print (f'Occupancy map at point object {i}-> pixels ',origin_map_img[1]+ round(pose[1]/pix_per_m),origin_map_img[0]+ round(pose[0]/pix_per_m), img_map[origin_map_img[1]+ round(pose[1]/pix_per_m),origin_map_img[0]+ round(pose[0]/pix_per_m)])
                    if img_map[origin_map_img[1]+ round(pose[1]/pix_per_m),origin_map_img[0]+ round(pose[0]/pix_per_m)]!=0:#### Yes axes seem to be "flipped" !=0:
                        print ('reject point, most likely part of arena, occupied inflated map')
                        tf_man.pub_static_tf(pos=[0,0,0], point_name=point_name, ref='head_rgbd_sensor_rgb_frame')
                        num_objs-=1
                    print (f"object found at robot coords.{pose} ")
                
            
            img=bridge.imgmsg_to_cv2(res.im_out.image_msgs[0])
            cv2.imshow('our of res'  , img)
        if key=='q':
            rospy.signal_shutdown("User hit q key to quit.")



#cv2_img_depth = bridge.imgmsg_to_cv2(res.segmented_objects_array.table_objects_array[i].depth_image_array[j] )
#cv2_img = bridge.imgmsg_to_cv2(res.segmented_objects_array.table_objects_array[i].rgb_image_array[j] )


def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a uniques
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    global tf_listener, ptcld_lis
    #rospy.init_node('image_tag_rgbd', anonymous=True)
    
    tf_listener = tf.TransformListener()
   

    ptcld_lis=rospy.Subscriber("/hsrb/head_rgbd_sensor/depth_registered/rectified_points",PointCloud2, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()





    #rgbd= RGBD()
    #rospy.wait_for_service('classify')
    #try:
    #    classify = rospy.ServiceProxy('/classify', Classify)    
    #except rospy.ServiceException as e:
    #    print("Service call failed: %s"%e)
    #rospy.Subscriber("/hsrb/head_rgbd_sensor/rgb/image_rect_color", Image, callback)
    #rospy.Subscriber("/hsrb/head_rgbd_sensor/depth_registered/image", Image, callback)
    #images= message_filters.Subscriber("/hsrb/head_rgbd_sensor/rgb/image_rect_color",Image)
    #images= message_filters.Subscriber("/usb_cam/image_raw",Image)
    #points= message_filters.Subscriber("/hsrb/head_rgbd_sensor/depth_registered/rectified_points",PointCloud2)
    #message_filters.Subscriber("/hsrb/head_rgbd_sensor/depth_registered/image"     ,Image)
    #ats= message_filters.ApproximateTimeSynchronizer([symbol,odom,twist],queue_size=5,slop=.1,allow_headerless=True)
    #ats= message_filters.ApproximateTimeSynchronizer([images,points],queue_size=5,slop=.1,allow_headerless=True)
    #ats.registerCallback(callback)
    #rospy.Subscriber("/hsrb/base_scan", LaserScan, callback)
    


if __name__ == '__main__':
    
    
    
    listener()


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
import rospkg

from smach_utils2 import *

from object_classification.srv import *
#from utils_srv import RGBD


from std_msgs.msg import String
first= True
rospack = rospkg.RosPack()
yaml_file = rospy.get_param("segmentation_params", "/segmentation_params.yaml")
#yaml_file = "/segmentation_params.yaml"
file_path = rospack.get_path('segmentation')+'/config_files'  + yaml_file





def read_segmentation_yaml(yaml_file = "/segmentation_params.yaml"):
    
    file_path = rospack.get_path('segmentation')+'/config_files'  + yaml_file

    with open(file_path, 'r') as file:
        content = yaml.safe_load(file)
    return content

#------------------------------------------------------
def nothing(x):
    pass

##############################################################################################################################################################################################################################################################################################################################
def callback(points_msg):

    global first , rospack , file_path
 

    print("DENTRO")
    #print('got imgs msgs')
    points_data = ros_numpy.numpify(points_msg)    
    image_data = points_data['rgb'].view((np.uint8, 4))[..., [2, 1, 0]]   #JUST TO MANTAIN DISPLAY
    image=cv2.cvtColor(image_data, cv2.COLOR_BGR2RGB)




    img=rgbd.get_image()
    img=cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
   

    cv2.imshow('xtion rgb'	, image)

     
    if first:
        print (first)
        cv2.imshow('class rgbd'  , img)
        df = read_segmentation_yaml()

        cv2.createTrackbar('Max Area', 'class rgbd', 0, 480*640, nothing)   ### AREA MAX IS THE WHOLE IMAGE 
        cv2.createTrackbar('Min Area', 'class rgbd', 0, 2000, nothing)   ### AREA MAX IS THE WHOLE IMAGE 
        cv2.createTrackbar('Hi limit pix y', 'class rgbd',240,480,nothing)
        cv2.createTrackbar('Lo limit pix y', 'class rgbd',0,240,nothing)
        cv2.setTrackbarPos('Max Area', 'class rgbd',df['higher']) 
        cv2.setTrackbarPos('Min Area', 'class rgbd',df['lower']) 
        cv2.setTrackbarPos('Hi limit pix y','class rgbd',df['reg_hy']) 
        cv2.setTrackbarPos('Lo limit pix y','class rgbd',df['reg_ly']) 
        first=False
    cv2.imshow('class rgbd'  , img)
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
            
            
        if key=='y': 
            
            
            print('file_path',file_path)
            df = read_segmentation_yaml()
            print('df',df)
            r = cv2.getTrackbarPos('Max Area', 'class rgbd')
            df['higher']=r
            r = cv2.getTrackbarPos('Min Area', 'class rgbd')
            df['lower']=r
            r = cv2.getTrackbarPos('Hi limit pix y', 'class rgbd')
            df['reg_hy']=r
            r = cv2.getTrackbarPos('Lo limit pix y', 'class rgbd')
            df['reg_ly']=r


            with open(file_path, 'w') as file:
                documents = yaml.dump(df, file, default_flow_style=False)
            return True

            
        


        if key=='s': 

            r = cv2.getTrackbarPos('Max Area', 'class rgbd')
            print ('#################################',r)
            
            head.set_joint_values([ 0.1, -0.5])
            res=segmentation_server.call()
            print( 'segmenting')
            brazo.set_named_target('go')
            origin_map_img=[round(img_map.shape[0]*0.5) ,round(img_map.shape[1]*0.5)]   
            if len(res.poses.data)==0:print('no objs')
            else:
                print ('QUATS PCA',res.quats.data)
                poses=np.asarray(res.poses.data)
                quats=np.asarray(res.quats.data)
                poses=poses.reshape((int(len(poses)/3) ,3     )      )  
                quats=quats.reshape((int(len(quats)/4) ,4     )      )  
                num_objs=len(poses)
                q=  quats[0]/np.linalg.norm(quats[0])
                print ('num_objs', num_objs, q)
                for i,pose in enumerate(poses):
                    #print (f'Occupancy map at point object {i}-> pixels ',origin_map_img[1]+ round(pose[1]/pix_per_m),origin_map_img[0]+ round(pose[0]/pix_per_m), img_map[origin_map_img[1]+ round(pose[1]/pix_per_m),origin_map_img[0]+ round(pose[0]/pix_per_m)])
                    point_name=f'object_{i}'
                    tf_man.pub_tf(pos=pose, rot =q , point_name=point_name+'_PCA', ref='head_rgbd_sensor_rgb_frame')## which object to choose   #TODO
                    tf_man.pub_static_tf(pos=pose, rot =q , point_name=point_name, ref='head_rgbd_sensor_rgb_frame')## which object to choose   #TODO
                    rospy.sleep(0.3)
                    tf_man.change_ref_frame_tf(point_name=point_name, new_frame='map')
                    rospy.sleep(0.3)
                    #pose,_= tf_man.getTF(point_name)
                    #print (f'Occupancy map at point object {i}-> pixels ',origin_map_img[1]+ round(pose[1]/pix_per_m),origin_map_img[0]+ round(pose[0]/pix_per_m), img_map[origin_map_img[1]+ round(pose[1]/pix_per_m),origin_map_img[0]+ round(pose[0]/pix_per_m)])
                    #if img_map[origin_map_img[1]+ round(pose[1]/pix_per_m),origin_map_img[0]+ round(pose[0]/pix_per_m)]!=0:#### Yes axes seem to be "flipped" !=0:
                    #    print ('reject point, most likely part of arena, occupied inflated map')
                    #    tf_man.pub_static_tf(pos=[0,0,0], point_name=point_name, ref='head_rgbd_sensor_rgb_frame')
                    #    num_objs-=1
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


#!/usr/bin/python
# -*- coding: utf-8 -*-
# Copyright (C) 2017 Toyota Motor Corporation
import rospy
import cv2
import ros_numpy
import numpy as np
from tmc_tabletop_segmentator.srv import TabletopSegmentation
from tmc_tabletop_segmentator.srv import TabletopSegmentationRequest
from sensor_msgs.msg import Image
from std_msgs.msg import String
import tf
import tf2_ros
import geometry_msgs.msg

from cv_bridge import CvBridge, CvBridgeError
bridge = CvBridge()


rospy.init_node('hsrb_tabletop_segmentation')
cv2_img=np.zeros((49,49))

tf_listener = tf.TransformListener()
tf_broadcaster = tf2_ros.TransformBroadcaster()#tf_broadcaster1=tf.TransformBroadcaster()
tf_static_broadcaster = tf2_ros.StaticTransformBroadcaster()#tf_broadcaster1=tf.TransformBroadcaster()
tfBuffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tfBuffer)

service_client = rospy.ServiceProxy(
    '/tabletop_segmentator_node/execute', TabletopSegmentation)
service_client.wait_for_service(timeout=1.0)

req = TabletopSegmentationRequest()
req.crop_enabled = True  # limit the processing area
req.crop_x_max = 0.7     # X coordinate maximum value in the area [m]
req.crop_x_min = -0.7    # X coordinate minimum value in the area [m]
req.crop_y_max = 1.0     # Y coordinate maximum value in the area [m]
req.crop_y_min = -1.0    # Y coordinate minimum value in the area [m]
req.crop_z_max = 1.1     # Z coordinate maximum value in the area [m]
req.crop_z_min = 0.0     # Z coordinate minimum value in the area [m]
req.cluster_z_max = 1.0  # maximum height value of cluster on table [m]
req.cluster_z_min = 0.0  # minimum height value of cluster on table [m]
req.remove_bg = False    # remove the background of the segment image

res = service_client(req)
objs_depth_centroids=[]
for i in range (len(res.segmented_objects_array.table_objects_array	)):
	print ( 'Plane',i,'has', len(res.segmented_objects_array.table_objects_array[i].depth_image_array), 'objects')
	for j in range (len(res.segmented_objects_array.table_objects_array[i].points_array)):
		cv2_img_depth = bridge.imgmsg_to_cv2(res.segmented_objects_array.table_objects_array[i].depth_image_array[0] )
		cv2_img = bridge.imgmsg_to_cv2(res.segmented_objects_array.table_objects_array[i].rgb_image_array[0] )
		pc= ros_numpy.numpify (res.segmented_objects_array.table_objects_array[i].points_array[j])
		points=np.zeros((pc.shape[0],3))
		points[:,0]=pc['x']
		points[:,1]=pc['y']
		points[:,2]=pc['z']
		objs_depth_centroids.append(np.mean(points,axis=0))
		print(i,j)
	    
		

for i in range (len(res.table_array.tables)):	
	print (res.table_array.tables[i].pose)

rospy.loginfo('Number of detected objects={0}'.format(
    len(   objs_depth_centroids)))
rospy.loginfo('Number of detected planes={0}'.format(
    len(res.table_array.tables)))
#(trans,rot)=tf_listener.lookupTransform('hand_palm_link', 'map', rospy.Time(0)) 
cv2.imshow("rgb", cv2_img) 
print len(objs_depth_centroids)
static_transformStamped = geometry_msgs.msg.TransformStamped()

while True:
	
	for ind, xyz in enumerate(objs_depth_centroids):
	
		#tf_broadcaster1.sendTransform( (xyz[0],xyz[1],xyz[2]),tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), "obj"+str(ind), "head_rgbd_sensor_link")
		static_transformStamped.header.stamp = rospy.Time.now()
		static_transformStamped.child_frame_id = "TF2"+(str)(ind)
		static_transformStamped.header.frame_id = "head_rgbd_sensor_link"
		static_transformStamped.child_frame_id = "TF2"+(str)(ind)
		static_transformStamped.transform.translation.x = float(xyz[0])
		static_transformStamped.transform.translation.z = float(xyz[2])
		static_transformStamped.transform.translation.y = float(xyz[1])
		quat = tf.transformations.quaternion_from_euler(0,0,0)
		static_transformStamped.transform.rotation.x = quat[0]
		static_transformStamped.transform.rotation.y = quat[1]
		static_transformStamped.transform.rotation.z = quat[2]
		static_transformStamped.transform.rotation.w = quat[3]
	
		tf_broadcaster.sendTransform(static_transformStamped)

	
	for ind, xyz in enumerate(objs_depth_centroids):
		
		rospy.sleep(.5)
		trans = tfBuffer.lookup_transform( 'odom',"TF2"+(str)(ind), rospy.Time())
		trans.header.frame_id='odom'
		trans.child_frame_id = "AA_ODOM_TF2"+(str)(ind)
		tf_static_broadcaster.sendTransform(trans)


	





	k = cv2.waitKey(1)
	if k & 0xFF == ord('q'):
	    # q key pressed so quit
	    print("Quitting...")

	    cv2.destroyAllWindows()
	    exit()

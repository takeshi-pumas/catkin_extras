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

from cv_bridge import CvBridge, CvBridgeError
bridge = CvBridge()


rospy.init_node('hsrb_tabletop_segmentation')

service_client = rospy.ServiceProxy(
    '/tabletop_segmentator_node/execute', TabletopSegmentation)
service_client.wait_for_service(timeout=1.0)

req = TabletopSegmentationRequest()
req.crop_enabled = True  # limit the processing area
req.crop_x_max = 1.0     # X coordinate maximum value in the area [m]
req.crop_x_min = -1.0    # X coordinate minimum value in the area [m]
req.crop_y_max = 1.0     # Y coordinate maximum value in the area [m]
req.crop_y_min = -1.0    # Y coordinate minimum value in the area [m]
req.crop_z_max = 1.8     # Z coordinate maximum value in the area [m]
req.crop_z_min = 0.8     # Z coordinate minimum value in the area [m]
req.cluster_z_max = 1.0  # maximum height value of cluster on table [m]
req.cluster_z_min = 0.0  # minimum height value of cluster on table [m]
req.remove_bg = False    # remove the background of the segment image

res = service_client(req)

for i in range (len(res.segmented_objects_array.table_objects_array	)):
	print ( 'UTUT',i, len(res.segmented_objects_array.table_objects_array[i].depth_image_array), len(res.segmented_objects_array.table_objects_array[i].rgb_image_array))
	if (len(res.segmented_objects_array.table_objects_array[i].rgb_image_array)>0):
		cv2_img_depth = bridge.imgmsg_to_cv2(res.segmented_objects_array.table_objects_array[i].depth_image_array[0] )
		cv2_img = bridge.imgmsg_to_cv2(res.segmented_objects_array.table_objects_array[i].rgb_image_array[0] )
		pc= ros_numpy.numpify (res.segmented_objects_array.table_objects_array[i].points_array[0])
		points=np.zeros((pc.shape[0],3))
		points[:,0]=pc['x']
		points[:,1]=pc['y']
		points[:,2]=pc['z']
	    
		print ('jala',points)
		   
for i in range (len(res.table_array.tables)):
	print (res.table_array.tables[i].pose)
rospy.loginfo('Number of detected objects={0}'.format(
    len(res.segmented_objects_array.table_objects_array)))
rospy.loginfo('Number of detected planes={0}'.format(
    len(res.table_array.tables)))

cv2.imshow("rgb", cv2_img) 
cv2.imshow("depth", cv2_img_depth) 
while True:
	k = cv2.waitKey(1)
	if k & 0xFF == ord('q'):
	    # q key pressed so quit
	    print("Quitting...")

	    cv2.destroyAllWindows()
	    exit()

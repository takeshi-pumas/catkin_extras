#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge
from erasers_hmi_ros.srv import ShowImageDisplayRequest, ShowImageDisplay

bridge = CvBridge()

rospy.init_node('debug_display_image')

show_img = rospy.ServiceProxy('show_image_to_display', ShowImageDisplay)
rospy.loginfo('waiting for service')
rospy.wait_for_service('show_image_to_display')

img = cv2.imread('./image.jpg')
cv2.imshow('window', img)
cv2.waitKey(100)

imgmsg = bridge.cv2_to_imgmsg(img, 'bgr8')

try:
    d = ShowImageDisplayRequest()
    d.image = imgmsg
    d.time = 3
    show_img(d)
    print('show image has done')
except:
    rospy.logerr("Service call failed: %s" % e)





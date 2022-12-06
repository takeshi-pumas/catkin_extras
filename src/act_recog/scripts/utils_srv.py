#!/usr/bin/env python3

from cv_bridge import CvBridge
from sensor_msgs.msg import Image 
import rospy
import tf
import tf2_ros

class RGB():
    def __init__(self):
        self._img_sub = rospy.Subscriber(
            #"/hsrb/head_rgbd_sensor/rgb/image_rect_color",     #FOR DEBUG USB CAM"/usb_cam/image_raw"
            "/usb_cam/image_raw",                               #"/hsrb/head_rgbd_sensor/rgb/image_rect_color"
            Image, self._img_cb)
        self._image_data = None
        
    def _img_cb(self, msg):
        global bridge
        self._image_data = bridge.imgmsg_to_cv2(msg)
        return
    
    def get_image(self):
        return self._image_data


#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String

from erasers_hmi_ros.srv import ShowImageDisplay, ShowImageDisplayResponse

class ErasersHmiNode:
    def __init__(self):
        #self.showing_flag = False

        s = rospy.Service('show_image_to_display', ShowImageDisplay, self.store_image)

        self.image_pub = rospy.Publisher('display_image', Image, queue_size=1)
        self.select_img_topic_pub = rospy.Publisher('display_image_topic', String, queue_size=1)
        
    def store_image(self, req):
        stored_image = req.image
        show_time = req.time

        while not rospy.is_shutdown():
            rospy.loginfo("Task timer subscriber connected {}".format(self.select_img_topic_pub.get_num_connections()))
            if self.select_img_topic_pub.get_num_connections() == 1:
                break

        #self.showing_flag = False
        #rate = rospy.Rate(10)
        #for _ in range(2):
        #    rate.sleep()
        
        self.select_img_topic_pub.publish(String('display_image'))
        self.pub_to_display(stored_image, show_time)
        return ShowImageDisplayResponse()

    def pub_to_display(self, stored_image, show_time):
        #self.showing_flag = True
        
        rate = rospy.Rate(10)
        exit_time = rospy.Time.now().secs + show_time
        
        while rospy.Time.now().secs < exit_time: #and self.showing_flag:
            self.image_pub.publish(stored_image)
            rate.sleep()

        #self.showing_flag = False

if __name__ == "__main__":
    rospy.init_node('erasers_hmi_ctrl_node')

    c = ErasersHmiNode()

    rospy.spin()

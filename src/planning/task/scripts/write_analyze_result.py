#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from utils.know_utils import *

def callback(msg):
    #rospy.loginfo("Mensaje recibido: %s", msg.data)
    takeshi_line = msg.data
    if len(takeshi_line) > 2:
        name = takeshi_line.split()[0]
        add_description(name, takeshi_line)

def listener():
    rospy.init_node('result_writer_node', anonymous=True)
    rospy.Subscriber('/analyze_result', String, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()

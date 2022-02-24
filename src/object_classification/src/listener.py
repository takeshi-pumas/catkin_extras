#!/usr/bin/env python
PKG = 'object_classification'
import roslib; roslib.load_manifest(PKG)

import rospy
from rospy.numpy_msg import numpy_msg
from object_classification.msg import Floats 

def callback(data):
    print rospy.get_name(), "I heard %s"%str(data.data)
    

def listener():
    rospy.init_node('listener')
    rospy.Subscriber("floats", numpy_msg(Floats), callback)
    rospy.spin()

if __name__ == '__main__':
    listener()

#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
from object_classification.srv import *

def classification_client(x):
    rospy.wait_for_service('classify')
    try:
        add_two_ints = rospy.ServiceProxy('/classify', Classify)
        resp1 = add_two_ints(x)
        print (resp1.out)
        return resp1
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 2:
        x = int(sys.argv[1])
        
    else:
        print(usage())
        sys.exit(1)
    print("Requesting %s"%(x))
    print(" %s  %s"%(x, classification_client(x)))
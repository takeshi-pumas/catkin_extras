#! /usr/bin/env python3

                                                 
from utils_pointing import *



def trigger_response(request):
    
    print ('Segmenting')
    points_msg=rospy.wait_for_message("/hsrb/head_rgbd_sensor/depth_registered/rectified_points",PointCloud2,timeout=5)
  

    #res= detect_human(points_msg)
    res= detect_pointing(points_msg)
    # IN PROGRESS
    #res = detect_all(points_msg)
    print (res)
    return res
    

rospy.loginfo("human detection service available")                    # initialize a ROS node
my_service = rospy.Service(                        # create a service, specifying its name,
    '/detect_pointing', Point_detector, trigger_response         # type, and callback
)
rospy.spin()   

#! /usr/bin/env python3

                                                 
from utils_pointing import *



def trigger_response(request):    
    print ('Segmenting for Pointing')
    points_msg=rospy.wait_for_message("/hsrb/head_rgbd_sensor/depth_registered/rectified_points",PointCloud2,timeout=25)
    
    dist = 6 if request.dist == 0 else request.dist
    res= detect_pointing(points_msg, dist, request.removeBKG)
    
    return res
    
def callback(request):
	print ('Segmenting for HUMAN DETECTOR')  
	points_msg=rospy.wait_for_message("/hsrb/head_rgbd_sensor/depth_registered/rectified_points",PointCloud2,timeout=25)
	dist = 6 if request.dist == 0 else request.dist
	#print("\n\nDISTANCIA",dist,"\n\n")
	res= detect_human(points_msg, dist, request.removeBKG)	
	return res    

rospy.loginfo("human pose detection service available")                    # initialize a ROS node
my_service = rospy.Service(                        # create a service, specifying its name,
    '/detect_pointing', Point_detector, trigger_response         # type, and callback
)
service2 = rospy.Service('/detect_human', Human_detector, callback ) 

rospy.spin()   

#! /usr/bin/env python3

                                                 
from utils import *



def trigger_response(request):
    ''' 
    Callback function used by the service server to process
    requests from clients. It returns a TriggerResponse
    '''
    print ('Segmenting')
    points_msg=rospy.wait_for_message("/hsrb/head_rgbd_sensor/depth_registered/rectified_points",PointCloud2,timeout=5)
    points_data = ros_numpy.numpify(points_msg)    
    image_data = points_data['rgb'].view((np.uint8, 4))[..., [2, 1, 0]]   
    image=cv2.cvtColor(image_data, cv2.COLOR_BGR2RGB)
    print (image.shape)
    
    
    cents,xyz, images, img = plane_seg( points_msg,lower=100    , higher=4000,reg_hy=350)
    
    print(len(cents))


    for i,cent in enumerate(cents):
                print (cent)
                x,y,z=cent
                if np.isnan(x) or np.isnan(y) or np.isnan(z):
                    print('nan')
                else:
                    t=write_tf(    (x,y,z),(0,0,0,1), 'Object'+str(i), "head_rgbd_sensor_rgb_frame"     )
                    broadcaster.sendTransform(t)
                    #broadcaster.sendTransform((x,y,z),(0,0,0,1), rospy.Time.now(), 'Object'+str(i),"head_rgbd_sensor_rgb_frame")
            
    rospy.sleep(.5)
    cents_map=[]
    for i in range(len (cents)):
        #trans,rot=tf_listener.lookupTransform('map', 'Object'+str(i), rospy.Time(0))
        try:
            trans = tfBuffer.lookup_transform('map', 'Object'+str(i), rospy.Time(), rospy.Duration(1.0))
                        
            trans,rot=read_tf(trans)
            print ("############tf2",trans,rot)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print ( 'No TF FOUND')


        cents_map.append(trans)
    ccs_map=np.asarray(cents_map)
    pose, quat=Floats(),Floats()
    res= SegmentationResponse()
    #pose.data=cents_map
    pose.data=ccs_map.ravel()

    res.poses=pose
    # res.quats=pose
    return res
    

rospy.loginfo("segmentation service available")                    # initialize a ROS node
my_service = rospy.Service(                        # create a service, specifying its name,
    '/segment', Segmentation, trigger_response         # type, and callback
)
rospy.spin()   

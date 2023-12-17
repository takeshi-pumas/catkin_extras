#! /usr/bin/env python3

                                                 
from utils import *



def trigger_response(request):
    ''' 
    Callback function used by the service server to process
    requests from clients. It returns a TriggerResponse
    '''
    print ('Segmenting')
    points_msg=rospy.wait_for_message("/hsrb/head_rgbd_sensor/depth_registered/rectified_points",PointCloud2,timeout=5)
    # <<<<<<<<<<<<<<<<<<<<<<ANTERIOR SEGMENTADOR>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    # points_data = ros_numpy.numpify(points_msg)    
    # image_data = points_data['rgb'].view((np.uint8, 4))[..., [2, 1, 0]]   
    # image=cv2.cvtColor(image_data, cv2.COLOR_BGR2RGB)
    # print (image.shape)
    # cents,xyz, images, img = plane_seg( points_msg,lower=10    , higher=4000,reg_hy=350)
    # <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    plot_im=False
    df=read_yaml('/segmentation_params.yaml')
    higher_v=df['higher']  #Higher area limit 
    lower_v=df['lower']    #Lower Area Limit
    reg_hy_v=df['reg_hy']  #higher limit of pix y ( bottom part of img) 
    reg_ly_v=df['reg_ly']  #lower limit of pix y ( high part of img) 
    #print ( 'segmentation params ',df)
    cents,xyz, images, img, xyz_c= plane_seg(points_msg,lower=lower_v, higher=higher_v,reg_ly=reg_ly_v,reg_hy=reg_hy_v)
#                                               (points_msg,hg=0.85,lg=1.5,th_v=0.03,lower=1000 ,higher=50000,reg_ly= 30,reg_hy=600)


    print(len(cents))
    quats_pca=[]
    for i,cent in enumerate(cents):
        print (cent)
        x,y,z=cent
        if np.isnan(x) or np.isnan(y) or np.isnan(z):
            print('nan')
        else:
            print ('Estimated Height of the object ',max(xyz_c[i][:,2])-min(xyz_c[i][:,2]))
            
            #np.save( "/home/takeshi/Documents/points", xyz_c[i]   )#### CONVENIENT FOR DEBUG
            points = xyz_c[i]
            points.shape
            E_R= points_to_PCA(points)
            print ('MATRIZ ROTACION PCA' ,E_R)
            quat=tf.transformations.quaternion_from_matrix(E_R) ### ROTACION expresada en quaternion de la rotacion propuesta por PCA
            print ('quat from R',quat)
            quats_pca.append(quat)
            #quat[0],quat[1],quat[2],quat[3]
            t=write_tf(    (x,y,z), (0,0,0,1), 'Object'+str(i), "head_rgbd_sensor_rgb_frame"   )

            #t=write_tf(    (x,y,z), (0,0,0,1), 'Corrected?'+str(i), "head_rgbd_sensor_rgb_frame"   )
            broadcaster.sendTransform(t)
   
                #ccs_map=np.asarray(cents_map)         
            """#broadcaster.sendTransform((x,y,z),(0,0,0,1), rospy.Time.now(), 'Object'+str(i),"head_rgbd_sensor_rgb_frame")
                        
                #trans,rot=tf_listener.lookupTransform('map', 'Object'+str(i), rospy.Time(0))"""
    """for i,cent in enumerate(cents):
                    try:
                        trans = tfBuffer.lookup_transform('map', 'Object'+str(i), rospy.Time(), rospy.Duration(5.0))
                        print ("############tf2",trans,rot)
                        cents_map.append(trans)
                        print ("############_APP",trans,rot,cents_map)
                        trans,rot=read_tf(trans)
                    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                            print ( 'No  object TF FOUND')"""
                #print ('#############',cents_map)
                #ccs_map=np.asarray(cents_map)
    if plot_im:
        cv2.imshow("SEG",img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    pose, quats=Floats(),Floats()
    res= SegmentationResponse()
    #pose.data=cents_map
    img_msg=bridge.cv2_to_imgmsg(img)
    if len(res.im_out.image_msgs)==0:
        res.im_out.image_msgs.append(img_msg)
    pose.data=np.asarray(cents).ravel()
    quats.data=np.asarray(quats_pca).ravel()
    #print ('##POSE',pose,trans,ccs_map,cents_map    )
    res.poses=pose
    res.quats=quats
    return res
    

rospy.loginfo("segmentation service available")                    # initialize a ROS node
my_service = rospy.Service(                        # create a service, specifying its name,
    '/segment', Segmentation, trigger_response         # type, and callback
)
rospy.spin()   

#! /usr/bin/env python3

                                                 
from utils import *



def trigger_response(request):
    ''' 
    Trigger service ( a null request performs segmentation)
    '''
    print ('Segmenting')
    points_msg=rospy.wait_for_message("/hsrb/head_rgbd_sensor/depth_registered/rectified_points",PointCloud2,timeout=5)
    df=read_yaml('/segmentation_params.yaml')
    higher_v=df['higher']  #Higher area limit 
    lower_v=df['lower']    #Lower Area Limit
    reg_hy_v=df['reg_hy']  #higher limit of pix y ( bottom part of img) 
    reg_ly_v=df['reg_ly']  #lower limit of pix y ( high part of img) 
    #print ( 'segmentation params ',df)
    cents,xyz, images, img, xyz_c= plane_seg(points_msg,lower=lower_v, higher=higher_v,reg_ly=reg_ly_v,reg_hy=reg_hy_v)
#  


    print(f'{len(cents)} centroids segmentated')
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
            t=write_tf(    (x,y,z), (0,0,0,1), 'Object'+str(i), "head_rgbd_sensor_rgb_frame"   )
            broadcaster.sendTransform(t)
   
    
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

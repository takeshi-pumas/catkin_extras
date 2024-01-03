#! /usr/bin/env python3

                                                 
from utils import *



def trigger_response(request):
    ''' 
    Trigger service ( a null request performs segmentation)
    '''
    print (f'Segmenting at {request.height.data: .2f} ')
    points_msg=rospy.wait_for_message("/hsrb/head_rgbd_sensor/depth_registered/rectified_points",PointCloud2,timeout=5)
    df=read_yaml('/segmentation_params.yaml')
    higher_v=df['higher']  #Higher area limit 
    lower_v=df['lower']    #Lower Area Limit
    reg_hy_v=df['reg_hy']  #higher limit of pix y ( bottom part of img) 
    reg_ly_v=df['reg_ly']  #lower limit of pix y ( high part of img) 
    #print ( 'segmentation params ',df)
    cents,xyz, images, img, xyz_c , cents_c = plane_seg(points_msg,lower=lower_v, higher=higher_v,reg_ly=reg_ly_v,reg_hy=reg_hy_v, plane_height=request.height.data)
#  print(f'{len(cents)} centroids segmentated')
    quats_pca=[]
    heights=[]
    widths=[]
    for i,cent in enumerate(cents):
        print (cent)
        x,y,z=cent
        if np.isnan(x) or np.isnan(y) or np.isnan(z):
            print('nan')
        else:
            height=max(xyz_c[i][:,2])-min(xyz_c[i][:,2])
            width = max(xyz_c[i][:,1])-min(xyz_c[i][:,1])
            print ('Estimated Height of the object ', height)
            print ('Estimated Width of the object ', width)
            heights.append(height)
            widths.append(width)
            
            np.save( "/home/roboworks/Documents/points", xyz_c[i]   )#### CONVENIENT FOR DEBUG
            
            points = xyz_c[i]
            #########################
            
            df_pts=pd.DataFrame(points)
            df_pts.columns=[['x','y','z']]
            threshold= df_pts['z'].min().values[0]*0.998  # REMOVE SOME GROUND points
            print (threshold)
            #threshsold=-0.978
            rslt_df_pts = df_pts[['x','y','z']][df_pts[['x','y','z']] > threshold]
            print (rslt_df_pts.describe())

            newpoints=rslt_df_pts[['x','y','z']].values
            E_R=points_to_PCA(newpoints)
            print(np.rad2deg(tf.transformations.euler_from_matrix(E_R)))
            print('###############ESTIAMTED EULER IN DEG',np.sign(np.rad2deg(tf.transformations.euler_from_matrix(E_R))[0])*np.rad2deg(tf.transformations.euler_from_matrix(E_R))[1])
            e_ER=tf.transformations.euler_from_matrix(E_R)
            ####quat=tf.transformations.quaternion_from_matrix(E_R) ### ROTACION expresada en quaternion de la rotacion propuesta por PCA### NO SE PORQUE ESTO NO JALA!
            quat= tf. transformations.quaternion_from_euler(e_ER[0],e_ER[1],e_ER[2])
            quats_pca.append(quat)
            t=write_tf(    (x,y,z), quat, 'Object'+str(i), "head_rgbd_sensor_rgb_frame"   )
            broadcaster.sendTransform(t)
   
    
    pose, quats=Floats(),Floats()
    pose_c= Floats()
    heights, widths =Floats(),Floats()
    res= SegmentationResponse()
    #pose.data=cents_map
    img_msg=bridge.cv2_to_imgmsg(img)
    if len(res.im_out.image_msgs)==0:
        res.im_out.image_msgs.append(img_msg)
    pose.data=np.asarray(cents).ravel()
    pose_c.data=np.asarray(cents_c).ravel()
    quats.data=np.asarray(quats_pca).ravel()
    res.poses=pose
    res.quats=quats
    res.poses_corr=pose_c
    
    return res
    

rospy.loginfo("segmentation service available")                    # initialize a ROS node
my_service = rospy.Service(                        # create a service, specifying its name,
    '/segment', Segmentation, trigger_response         # type, and callback
)
rospy.spin()   

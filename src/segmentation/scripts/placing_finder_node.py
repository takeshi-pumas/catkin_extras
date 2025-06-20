#! /usr/bin/env python3

                                                 
from utils_placing import *



def trigger_response(request):
    ''' 
    Trigger service ( a null request performs segmentation)
    '''
    print (f'Segmenting at {request.height.data: .2f} ')
    points_msg=rospy.wait_for_message("/hsrb/head_rgbd_sensor/depth_registered/rectified_points",PointCloud2,timeout=5)
    points_data = ros_numpy.numpify(points_msg)    
    image_data = points_data['rgb'].view((np.uint8, 4))[..., [2, 1, 0]]   
    image=cv2.cvtColor(image_data, cv2.COLOR_BGR2RGB)
    image = points_data['rgb'].view((np.uint8, 4))[..., [2, 1, 0]]
    rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)    
    image_height, image_width, _ = rgb_image.shape
    #
    ################
    #CORRECT POINTS###################
    ################
    try:
            trans = tfBuffer.lookup_transform('map', 'head_rgbd_sensor_link', rospy.Time())
                        
            trans,rot=read_tf(trans)
            #print ("############head",trans,rot)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print ( 'No head TF FOUND')
    t= write_tf(trans,rot)
    cloud_out = do_transform_cloud(points_msg, t)
    np_corrected=ros_numpy.numpify(cloud_out)
    corrected=np_corrected.reshape(points_data.shape)
    zs_no_nans=corrected['z'][~np.isnan(corrected['z'])]
    planes =[]

    #AUTO DETECT PLANE?
    if request.height.data==-1:
        xyz = np.vstack((np_corrected['x'], np_corrected['y'], np_corrected['z'])).T
 
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(xyz)
 
        # Apply RANSAC to segment the dominant plane
        planes=[]
        horizontal_plane_found = False
        while len(pcd.points) > 10 and not horizontal_plane_found:
            plane_model, inliers = pcd.segment_plane(distance_threshold=0.02,
                                                     ransac_n=3,
                                                     num_iterations=1000)
            [a, b, c, d] = plane_model  # Plane equation: ax + by + cz + d = 0wrt to sensor
            plane_coeffs = [a, b, c, d]
            print(f'Estimated Plane Coeffs in Sensor Frame: {plane_coeffs}')
            inlier_cloud = pcd.select_by_index(inliers)
            print(f"Detected Plane: {plane_model}")
                
            if abs(a) < 0.1 and abs(b) < 0.1:  # Horizontal plane condition
                print(f"Detected Horizontal Plane: {plane_model}")
                inlier_cloud = pcd.select_by_index(inliers)
                planes.append(plane_model)  # Store detected horizontal plane
                ransac_mask= np.zeros(corrected['z'].shape)#mask
                inlier_points = np.asarray(pcd.points)[inliers]
                mean_inlier = np.mean(inlier_points, axis=0)  # Mean across the points (axis=0 means mean of each coordinate)
                save_auto_plane_region(inlier_points)
                print("Mean of inliers:", mean_inlier)
                
                

                planes_heights=[mean_inlier[2]]
                
                if mean_inlier[2]>0.1:
                    horizontal_plane_found=True
                else:
                    pcd = pcd.select_by_index(inliers, invert=True)
                    print( 'Floor, trivial solution, lets keep looking')
                
                
            else:
                # If the plane is not horizontal, remove the inliers and continue searching for the next plane
                print("Detected a non-horizontal plane, removing inliers and trying again...")
                pcd = pcd.select_by_index(inliers, invert=True)  # Remove the inliers
            

            

    else:planes_heights=[request.height.data]
    for plane_height in planes_heights:
        low_plane = (corrected['z'] > (plane_height-0.02)) #plane height
        high_plane = (corrected['z'] < (plane_height+0.02))#plane height + obj height
        orig_image= rgb_image.copy()
        mask= np.zeros(corrected['z'].shape)#mask
        z_lims=np.logical_and(low_plane, high_plane)
        
        result_indices = np.where(z_lims)#np.logical_and(z_lims, x_lims))
        mask[result_indices]=200
        _, binary_image = cv2.threshold(mask, 20, 255, cv2.THRESH_BINARY)
        
        cv2_image = cv2.cvtColor(binary_image.astype(np.uint8), cv2.COLOR_GRAY2BGR) 
        orig_image=cv2.bitwise_and(orig_image, cv2_image)
        
        #######################################




        eroded_image=cv2.erode(binary_image,kernel = np.ones((40, 40), np.uint8))
        deb_image=np.copy(orig_image)
        print ('quick estimation',np.nanmean(corrected['x'][np.where(eroded_image==255)]),np.nanmean(corrected['y'][np.where(eroded_image==255)]),np.nanmean(corrected['z'][np.where(eroded_image==255)]))
        contours, hierarchy = cv2.findContours(eroded_image.astype(np.uint8) ,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        cc=[]
        for contour in contours:
            M = cv2.moments(contour)
            area = cv2.contourArea(contour)
            if area > 200:          
                print (area)      
            # calculate x,y coordinate of center
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                cc.append((cX,cY))
                deb_image=cv2.circle(orig_image, (cX, cY), 5, (200, 200, 0), -1)
            else:print ('area too small') 
        poses=[]
        for (cX,cY) in cc:
            x,y = cX,cY
            print (x,y,'x,y')
            pose=np.asarray((np.mean(corrected['x'][y:y+45,x-23:x+22]),
                             np.mean(corrected['y'][y:y+45,x-23:x+22]),
                             np.mean(corrected['z'][y:y+45,x-23:x+22]) + 0.05))    
            deb_image[y:y+45,x-23:x+22,0]=255
            poses.append(pose)
            print('estimation', pose)
    ###################################3
    pose, quats=Floats(),Floats()
    heights_res, widths_res=Floats(),Floats()
    pose_c= Floats()
    heights, widths =Floats(),Floats()
    res= SegmentationResponse()
    #############################################
    print (mask.shape, deb_image.shape)
    

    
    
    
    img_msg=bridge.cv2_to_imgmsg(deb_image)
    #plt.imshow(img)
    #plt.imshow (image_with_contours)
    res.im_out.image_msgs.append(img_msg)
    pose.data=np.asarray(poses).ravel()
    #quats.data=np.asarray(quats_pca).ravel()
    res.poses=pose
    #res.quats=quats    
    #quats.data=np.asarray(quats_pca).ravel()
    #widths_res=np.asarray(widths).ravel()
    #res.widths=widths_res
    #heights_res=np.asarray(heights).ravel()
    #res.heights=heights_res
    return res        

#rospy.init_node('placing_finder') 
rospy.loginfo("Placing Finder service available")                    # initialize a ROS node
my_service = rospy.Service(                        # create a service, specifying its name,
    '/placing_finder', Segmentation, trigger_response         # type, and callback
)
rospy.spin()   

# -*- coding: utf-8 -*-
#!/usr/bin/env python
    
import numpy as np
import rospy
import ros_numpy
import tf2_ros
import tf
import os
import message_filters
from sensor_msgs.msg import Image , LaserScan , PointCloud2
from geometry_msgs.msg import TransformStamped
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import rospkg
from tuner_utils import *
from object_classification.srv import *
#from utils_srv import RGBD
from std_msgs.msg import String
#to CLIP
import torch
import clip
from PIL import Image
global model , preprocess
device = "cuda" if torch.cuda.is_available() else "cpu"
model, preprocess = clip.load("ViT-B/32", device=device)

first= True
rospack = rospkg.RosPack()
yaml_file = rospy.get_param("segmentation_params", "/segmentation_params.yaml")
file_path = rospack.get_path('segmentation')+'/config_files'  + yaml_file

def read_segmentation_yaml(yaml_file = "/segmentation_params.yaml"):
    
    file_path = rospack.get_path('segmentation')+'/config_files'  + yaml_file

    with open(file_path, 'r') as file:
        content = yaml.safe_load(file)
    return content

#------------------------------------------------------
def nothing(x):
    pass

##############################################################################################################################################################################################################################################################################################################################
def callback(points_msg):
    global first , rospack , file_path
    #print('got imgs msgs')
    points_data = ros_numpy.numpify(points_msg)    
    image_data = points_data['rgb'].view((np.uint8, 4))[..., [2, 1, 0]]   #JUST TO MANTAIN DISPLAY
    image=cv2.cvtColor(image_data, cv2.COLOR_BGR2RGB)

    img=rgbd.get_image()
    img=cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
   
    #cv2.imshow('xtion rgb'	, image)
     
    if first:
        print (first)
        cv2.imshow('class rgbd'  , img)
        df = read_segmentation_yaml()

        cv2.createTrackbar('Max Area', 'class rgbd', 0, 240*320, nothing)   ### AREA MAX IS half THE WHOLE IMAGE 
        cv2.createTrackbar('Min Area', 'class rgbd', 0, 2000, nothing)  
        cv2.createTrackbar('Hi limit pix y', 'class rgbd',240,480,nothing)
        cv2.createTrackbar('Lo limit pix y', 'class rgbd',0,240,nothing)
        cv2.createTrackbar('Hi limit pix x', 'class rgbd',320,640,nothing)
        cv2.createTrackbar('Lo limit pix x', 'class rgbd',0,319,nothing)
        cv2.createTrackbar('Plane height (cms)', 'class rgbd', -10, 100, nothing)   ### AREA MAX IS half THE WHOLE IMAGE 
        cv2.setTrackbarPos('Max Area', 'class rgbd',df['higher']) 
        cv2.setTrackbarPos('Min Area', 'class rgbd',df['lower']) 
        cv2.setTrackbarPos('Hi limit pix y','class rgbd',df['reg_hy']) 
        cv2.setTrackbarPos('Lo limit pix y','class rgbd',df['reg_ly']) 
        cv2.setTrackbarPos('Hi limit pix x','class rgbd',df['reg_hx']) 
        cv2.setTrackbarPos('Lo limit pix x','class rgbd',df['reg_lx']) 
        first=False
    cv2.imshow('class rgbd' , img)
    #print (r)

    # Process any keyboard commands
    keystroke = cv2.waitKey(1)
    
    if 32 <= keystroke and keystroke < 128:
        key = chr(keystroke).lower()
        print (key)
        if key=='u': 

            print('cha')
            ptcld_lis.unregister()
            
            
        elif key=='f': 
            
            print('file_path',file_path)
            df = read_segmentation_yaml()
            print('df',df)
            r = cv2.getTrackbarPos('Max Area', 'class rgbd')
            df['higher']=r
            r = cv2.getTrackbarPos('Min Area', 'class rgbd')
            df['lower']=r
            r = cv2.getTrackbarPos('Hi limit pix y', 'class rgbd')
            df['reg_hy']=r
            r = cv2.getTrackbarPos('Lo limit pix y', 'class rgbd')
            df['reg_ly']=r
            r = cv2.getTrackbarPos('Hi limit pix x', 'class rgbd')
            df['reg_hx']=r
            r = cv2.getTrackbarPos('Lo limit pix x', 'class rgbd')
            df['reg_lx']=r


            with open(file_path, 'w') as file:
                documents = yaml.dump(df, file, default_flow_style=False)
            return True

        elif key=='h':
            print ('############# hand cam')           
            cv2.imshow('our of res'  , cv2.cvtColor(hand_rgb.get_image(), cv2.COLOR_BGR2RGB)   )

        elif key=='y':
            print ('#############YOLO SERVICE YCB REQUESTED')
            img_msg  = bridge.cv2_to_imgmsg(image)
            req      = classify_client.request_class()
            req.in_.image_msgs.append(img_msg)
            res      = classify_client(req)
            for i in range(len(res.poses)):
                tf_man.getTF("head_rgbd_sensor_rgb_frame")
                position = [res.poses[i].position.x ,res.poses[i].position.y,res.poses[i].position.z]
                print(position )
                tf_man.pub_static_tf(pos= position, rot=[0,0,0,1], ref="head_rgbd_sensor_rgb_frame", point_name=res.names[i].data[4:] )   
                rospy.sleep(0.3)
                tf_man.change_ref_frame_tf(res.names[i].data[4:])
            debug_image=bridge.imgmsg_to_cv2(res.debug_image.image_msgs[0])
            cv2.imshow('our of res'  , debug_image)
            save_image(debug_image,name="yolo_result")

        elif key=='p':
            print ('#############Pointing  SERVICE openPose REQUESTED')
            
            res=pointing_detect_server.call()
            print (f'xr{res.x_r} yr{res.y_r} xl{res.x_l} yl{res.y_l} ')
           
            if (res.x_r+res.y_r)!=0:
                print('right')
                tf_man.pub_static_tf(pos=[res.x_r, res.y_r,0], rot =[0,0,0,1], point_name='pointing_right')
            if (res.x_l+res.y_l)!=0:
                print('left')
                tf_man.pub_static_tf(pos=[res.x_l, res.y_l,0], rot =[0,0,0,1], point_name='pointing_left')
            debug_image=bridge.imgmsg_to_cv2(res.debug_image[0])
            cv2.imshow('our of res'  , debug_image)
            save_image(debug_image,name="openP_result")

        elif key=='s': 

            request= segmentation_server.request_class() 
            r = cv2.getTrackbarPos('Plane height (cms)', 'class rgbd')
            request.height.data=r * 0.01
            if r == 100:request.height.data=-1.0
            print ('#############Segmenting at plane####################',request.height.data)
            
            #head.set_joint_values([ 0.1, -0.5])
            res=segmentation_server.call(request)
            print (f'Planes candidates{res.planes.data}')
            succ=seg_res_tf(res)
            print (f'heights{res.heights.data}, widths {res.widths.data}')
            img=bridge.imgmsg_to_cv2(res.im_out.image_msgs[0])
            cv2.imshow('our of res'  , img)
            save_image(img,name="segment_result")

        elif key=='l': #PLACING FINDER

            request= segmentation_server.request_class() 
            r = cv2.getTrackbarPos('Plane height (cms)', 'class rgbd')
            request.height.data=r * 0.01
            if r == 100:request.height.data=-1.0
            print ('#############Finding placing in plane####################',request.height.data)
            
            #head.set_joint_values([ 0.1, -0.5])
            res=placing_finder_server.call(request)
            #succ=seg_res_tf(res)
            print (f'Placing Area at {res.poses.data}')
            tf_man.pub_static_tf(pos=[res.poses.data[0], res.poses.data[1],res.poses.data[2]], rot =[0,0,0,1], point_name='placing_area')
            img=bridge.imgmsg_to_cv2(res.im_out.image_msgs[0])
            cv2.imshow('our of res'  , img)
            save_image(img,name="placingFinder_result")
      
        elif key == 'i':        # obtener una imagen 'normal'
            img = hand_rgb.get_image()
            img = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)


            save_image(img,name="image",dirName="train")    # dirName requiere que la carpeta exista !!!
            #rospack.get_path("config_files")
            #cv2.imwrite("breakfast", rgbd.get_image())
        elif key == 'c':        # Usar CLIP para tarea by prompt
            img=rgbd.get_image()
            #cv2.imwrite('img.png',img)
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            cv2.imshow('our of res'  , img)
            print ('got image for CLIP analysis')
            keys=[ 'table','shelf','human']
            image = preprocess(Image.fromarray(img)).unsqueeze(0).to(device) #img array from grbd get image
            text = clip.tokenize(keys).to(device)

            with torch.no_grad():
                image_features = model.encode_image(image)
                text_features = model.encode_text(text)
                
                logits_per_image, logits_per_text = model(image, text)
                probs = logits_per_image.softmax(dim=-1).cpu().numpy()

            print("Label probs:", probs,keys[np.argmax(probs)] ,keys, probs[0][1] ) # prints: [[0.9927937  0.00421068 0.00299572]]

        elif key == 'r':
                       # Obtener una nube de puntos del sensor
            rospy.loginfo("Esperando nube de puntos desde /hsrb/head_rgbd_sensor/depth_registered/rectified_points")
            #pointcloud_msg = rospy.wait_for_message("/hsrb/head_rgbd_sensor/depth_registered/rectified_points", PointCloud2)
            pointcloud_msg = points_msg
            if pointcloud_msg is None:
                rospy.logerr("No se recibió la nube de puntos.")
                return
            rospy.loginfo("Nube de puntos recibida. Enviando solicitud de segmentación...")

            # Nombre de la región a segmentar (ajustar según el YAML)
            region_name = "beverage_area"
            
            rospy.wait_for_service("segment_region")
            try:
                request = SegmentRegionRequest(pointcloud=pointcloud_msg, region_name=region_name)
                response = segment_service(request)

                if response.success:
                    bridge = CvBridge()
                    mask = bridge.imgmsg_to_cv2(response.mask, "mono8")
                    #TODO:apply dilate and erode
                    # **Aplicar operaciones morfológicas para reducir ruido**
                    kernel = np.ones((13, 13), np.uint8)  # Define un kernel de 5x5 (ajustable)
                    
                    # mask_cleaned = cv2.dilate(mask, kernel, iterations=5)  # **Rellena huecos**
                    # mask_cleaned = cv2.erode(mask_cleaned, kernel, iterations=1)  # **Reduce pequeños artefactos**
                    
                    segment_img = cv2.bitwise_and(img, img, mask=mask)
                else:
                    rospy.logwarn("Error en segmentación: " + response.message)
                cv2.imshow("Mascara de Segmentacion", segment_img)
                cv2.waitKey(0)
                cv2.destroyAllWindows()
            except rospy.ServiceException as e:
                rospy.logerr("Error llamando al servicio: %s" % e)

        elif key == 'd':        # Usar Dino para object classification by prompt
            prompt = "coke"     #put here favorite drink
            img=rgbd.get_image()
            #cv2.imwrite('img.png',img)
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            # Convert image to ROS format
            ros_image = bridge.cv2_to_imgmsg(img, encoding="bgr8")

            # Create a proper ROS String message
            prompt_msg = String()
            prompt_msg.data = prompt

            rospy.wait_for_service('grounding_dino_detect')
            try:
                response = classify_client_dino(ros_image, prompt_msg)
                print("Result:", response.result.data,"for drink:",prompt)
                if response.image is None or response.image.data == b'':
                    print("Error: Received an empty image response!")
                else:
                    debug_image = bridge.imgmsg_to_cv2(response.image, desired_encoding="rgb8")
                    cv2.imshow('our of res', debug_image)
                    cv2.waitKey(1)  # Allow OpenCV to refresh window

            except rospy.ServiceException as e:
                print(f"Service call failed: {e}")



        elif key=='q':
            rospy.signal_shutdown("User hit q key to quit.")






def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a uniques
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    global tf_listener, ptcld_lis 
    
    #rospy.init_node('image_tag_rgbd', anonymous=True)
    
    tf_listener = tf.TransformListener() 
    ptcld_lis=rospy.Subscriber("/hsrb/head_rgbd_sensor/depth_registered/rectified_points",PointCloud2, callback)
    #ptcld_lis=rospy.Subscriber("/hsrb/head_r_stereo_camera/image_raw",PointCloud2, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()


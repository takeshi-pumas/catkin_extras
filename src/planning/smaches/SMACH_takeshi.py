#!/usr/bin/env python3
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist , PointStamped , Point
import actionlib
import smach
import smach_ros
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, Point , Quaternion ,WrenchStamped 
from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import String      
from tmc_msgs.msg import Voice
import moveit_commander
import moveit_msgs.msg
import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from object_classification.srv import *
#import face_recognition 
from face_recog.msg import *
from face_recog.srv import *
import cv2  
import rospy 
import numpy as np
from hmm_navigation.msg import NavigateActionGoal, NavigateAction
from cv_bridge import CvBridge, CvBridgeError
from grasp_utils import *
#from utils_notebooks import *
#from utils_takeshi import *
#from utils_srv import *  
from sensor_msgs.msg import Image , LaserScan , PointCloud2
from hri_msgs.msg import RecognizedSpeech
import tf
########################################################################
#THIS SMACH USES Face Recog
###########################################################################

########## Functions for takeshi states ##########
from tmc_msgs.msg import Voice
takeshi_talk_pub = rospy.Publisher('/talk_request', Voice, queue_size=10)
def string_to_Voice(sentence='I am ready to begin'):
    voice_message=Voice()
    voice_message.sentence = sentence
    voice_message.queueing = False
    voice_message.language = 1
    voice_message.interrupting = False

    return(voice_message)
    
def wait_for_push_hand(time=10):

    start_time = rospy.get_time()
    time= 10
    print('timeout will be ',time,'seconds')
    while rospy.get_time() - start_time < time:
        torque=rospy.wait_for_message("/hsrb/wrist_wrench/raw", WrenchStamped)
        if np.abs(torque.wrench.torque.y)>0.5:
            print(' Hand Pused Ready TO start')
            takeshi_talk_pub.publish(string_to_Voice())
            return True
            break



    if (rospy.get_time() - start_time >= time):
        print(time, 'secs have elapsed with no hand push')
        return False


class RGB():
    
    def __init__(self):
        self._img_sub = rospy.Subscriber(
            "/hsrb/head_rgbd_sensor/rgb/image_rect_color",     #FOR DEBUG USB CAM"/usb_cam/image_raw"
            #"/usb_cam/image_raw",                               #"/hsrb/head_rgbd_sensor/rgb/image_rect_color"
            Image, self._img_cb)
        
        self._image_data = None
        
    def _img_cb(self, msg):
        global bridge
        
        self._image_data = bridge.imgmsg_to_cv2(msg)
        
        return
    
    def get_image(self):
        
        return self._image_data

class Proto_state(smach.State):###example of a state definition.
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'])
        self.tries=0
    def execute(self,userdata):
        rospy.loginfo('State : PROTO_STATE')

        if self.tries==3:
            self.tries=0 
            return'tries'
        if succ:
            return 'succ'
        else:
            return 'failed'
        global trans_hand
        self.tries+=1
        if self.tries==3:
            self.tries=0 
            return'tries'
def res_to_array(res):
    xyz_wrt_robot=(res.message[1:-1].split(', ')[0:3])
    x_wrt_robot=(float)(xyz_wrt_robot[0])
    y_wrt_robot=(float)(xyz_wrt_robot[1])
    z_wrt_robot=(float)(xyz_wrt_robot[2])

    x_wrt_robot,y_wrt_robot,z_wrt_robot
    return np.asarray((x_wrt_robot,y_wrt_robot,z_wrt_robot))

def get_points_corrected():
    data = rospy.wait_for_message('/hsrb/head_rgbd_sensor/depth_registered/rectified_points', PointCloud2)
    np_data=ros_numpy.numpify(data)
    ##trans,rot=listener.lookupTransform('/map', '/head_rgbd_sensor_frame', rospy.Time(0))  #Robot real
    #trans,rot=listener.lookupTransform('/map', '/head_rgbd_sensor_gazebo_frame', rospy.Time(0))  #GAZEBO
    trans, rot = tf_man.getTF(target_frame='head_rgbd_sensor_frame', ref_frame='map')
    eu=np.asarray(tf.transformations.euler_from_quaternion(rot))
    t=TransformStamped()
    rot=tf.transformations.quaternion_from_euler(-eu[1],np.pi,0)
    t.header.stamp = data.header.stamp

    t.transform.rotation.x = rot[0]
    t.transform.rotation.y = rot[1]
    t.transform.rotation.z = rot[2]
    t.transform.rotation.w = rot[3]

    cloud_out = do_transform_cloud(data, t)
    np_corrected=ros_numpy.numpify(cloud_out)
    corrected=np_corrected.reshape(np_data.shape)
    return corrected

def move_abs(vx,vy,vw):
    twist = Twist()
    twist.linear.x = vx
    twist.linear.y = vy
    twist.angular.z = vw / 180.0 * np.pi  
    base_vel_pub.publish(twist)

def pose_2_np(wp_p):
    
    #Takes a pose message and returns array
   
    return np.asarray((wp_p.pose.position.x,wp_p.pose.position.y,wp_p.pose.position.z)) , np.asarray((wp_p.pose.orientation.w,wp_p.pose.orientation.x,wp_p.pose.orientation.y, wp_p.pose.orientation.z)) 
def np_2_pose(position,orientation):
    #Takes a pose np array and returns pose message
    wb_p= geometry_msgs.msg.PoseStamped()
    
    wb_p.pose.position.x= position[0]
    wb_p.pose.position.y= position[1]
    wb_p.pose.position.z= position[2]
    wb_p.pose.orientation.w= orientation[0]
    wb_p.pose.orientation.x= orientation[1]
    wb_p.pose.orientation.y= orientation[2]
    wb_p.pose.orientation.z= orientation[3]
    return wb_p

    """def open_gripper():
        target_motor=1
        gripper.set_start_state_to_current_state()
        try:
            gripper.set_joint_value_target({'hand_motor_joint':target_motor})
        except:
            print('OOB')
        succ=gripper.go()
    def close_gripper():
        target_motor=0.0
        gripper.set_start_state_to_current_state()
        try:
            gripper.set_joint_value_target({'hand_motor_joint':target_motor})
        except:
            print('OOB')
        succ=gripper.go()"""
def correct_points(low=.27,high=1000):

    #Corrects point clouds "perspective" i.e. Reference frame head is changed to reference frame map
    data = rospy.wait_for_message('/hsrb/head_rgbd_sensor/depth_registered/rectified_points', PointCloud2)
    np_data=ros_numpy.numpify(data)
    #trans,rot=listener.lookupTransform('/map', '/head_rgbd_sensor_gazebo_frame', rospy.Time(0)) 
    trans, rot = tf_man.getTF(target_frame='head_rgbd_sensor_gazebo_frame') #ref_frame is 'map' by default
    eu=np.asarray(tf.transformations.euler_from_quaternion(rot))
    t=TransformStamped()
    rot=tf.transformations.quaternion_from_euler(-eu[1],0,0)
    t.header.stamp = data.header.stamp
    
    t.transform.rotation.x = rot[0]
    t.transform.rotation.y = rot[1]
    t.transform.rotation.z = rot[2]
    t.transform.rotation.w = rot[3]

    cloud_out = do_transform_cloud(data, t)
    np_corrected=ros_numpy.numpify(cloud_out)
    corrected=np_corrected.reshape(np_data.shape)

    img= np.copy(corrected['y'])

    img[np.isnan(img)]=2
    img3 = np.where((img>low)&(img< 0.99*(trans[2])),img,255)
    return img3

def plane_seg_square_imgs(lower=500 ,higher=50000,reg_ly= 30,reg_hy=600,plt_images=True):

    #Segment  Plane using corrected point cloud
    #Lower, higher = min, max area of the box
    #reg_ly= 30,reg_hy=600    Region (low y  region high y ) Only centroids within region are accepted
    
    image= rgbd.get_h_image()
    iimmg= rgbd.get_image()
    points_data= rgbd.get_points()
    img=np.copy(image)
    img3= correct_points()


    contours, hierarchy = cv2.findContours(img3.astype('uint8'),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    i=0
    cents=[]
    points=[]
    images=[]
    for i, contour in enumerate(contours):

        area = cv2.contourArea(contour)

        if area > lower and area < higher :
            M = cv2.moments(contour)
            # calculate x,y coordinate of center
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])


            boundRect = cv2.boundingRect(contour)
            #just for drawing rect, dont waste too much time on this
            image_aux= iimmg[boundRect[1]:boundRect[1]+max(boundRect[2],boundRect[3]),boundRect[0]:boundRect[0]+max(boundRect[2],boundRect[3])]
            images.append(image_aux)
            img=cv2.rectangle(img,(boundRect[0], boundRect[1]),(boundRect[0]+boundRect[2], boundRect[1]+boundRect[3]), (0,0,0), 2)
            # calculate moments for each contour
            if (cY > reg_ly and cY < reg_hy  ):

                cv2.circle(img, (cX, cY), 5, (255, 255, 255), -1)
                cv2.putText(img, "centroid_"+str(i)+"_"+str(cX)+','+str(cY)    ,    (cX - 25, cY - 25)   ,cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 0), 2)
                print ('cX,cY',cX,cY)
                xyz=[]


                for jy in range (boundRect[0], boundRect[0]+boundRect[2]):
                    for ix in range(boundRect[1], boundRect[1]+boundRect[3]):
                        aux=(np.asarray((points_data['x'][ix,jy],points_data['y'][ix,jy],points_data['z'][ix,jy])))
                        if np.isnan(aux[0]) or np.isnan(aux[1]) or np.isnan(aux[2]):
                            'reject point'
                        else:
                            xyz.append(aux)

                xyz=np.asarray(xyz)
                cent=xyz.mean(axis=0)
                cents.append(cent)
                print (cent)
                points.append(xyz)
            else:
                print ('cent out of region... rejected')
    sub_plt=0
    if plt_images:
        for image in images:

            sub_plt+=1
            ax = plt.subplot(5, 5, sub_plt )

            plt.imshow(image)
            plt.axis("off")

    cents=np.asarray(cents)
    ### returns centroids found and a group of 3d coordinates that conform the centroid
    return(cents,np.asarray(points), images)
def seg_square_imgs(lower=2000,higher=50000,reg_ly=0,reg_hy=1000,reg_lx=0,reg_hx=1000,plt_images=False): 

    #Using kmeans for image segmentation find
    #Lower, higher = min, max area of the box
    #reg_ly= 30,reg_hy=600,reg_lx=0,reg_hx=1000,    Region (low  x,y  region high x,y ) Only centroids within region are accepted
    image= rgbd.get_h_image()
    iimmg= rgbd.get_image()
    points_data= rgbd.get_points()
    values=image.reshape((-1,3))
    values= np.float32(values)
    criteria= (  cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER  ,1000,0.1)
    k=6
    _ , labels , cc =cv2.kmeans(values , k ,None,criteria,30,cv2.KMEANS_RANDOM_CENTERS)
    cc=np.uint8(cc)
    segmented_image= cc[labels.flatten()]
    segmented_image=segmented_image.reshape(image.shape)
    th3 = cv2.adaptiveThreshold(segmented_image,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,11,2)
    kernel = np.ones((5,5),np.uint8)
    im4=cv2.erode(th3,kernel,iterations=4)
    plane_mask=points_data['z']
    cv2_img=plane_mask.astype('uint8')
    img=im4
    contours, hierarchy = cv2.findContours(im4.astype('uint8'),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    i=0
    cents=[]
    points=[]
    images=[]
    for i, contour in enumerate(contours):

        area = cv2.contourArea(contour)

        if area > lower and area < higher :
            M = cv2.moments(contour)
            # calculate x,y coordinate of center
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])


            boundRect = cv2.boundingRect(contour)
            #just for drawing rect, dont waste too much time on this
            image_aux= iimmg[boundRect[1]:boundRect[1]+max(boundRect[3],boundRect[2]),boundRect[0]:boundRect[0]+max(boundRect[3],boundRect[2])]
            images.append(image_aux)
            img=cv2.rectangle(img,(boundRect[0], boundRect[1]),(boundRect[0]+boundRect[2], boundRect[1]+boundRect[3]), (0,0,0), 2)
            #img=cv2.rectangle(img,(boundRect[0], boundRect[1]),(boundRect[0]+max(boundRect[2],boundRect[3]), boundRect[1]+max(boundRect[2],boundRect[3])), (0,0,0), 2)
            # calculate moments for each contour
            if (cY > reg_ly and cY < reg_hy and  cX > reg_lx and cX < reg_hx   ):

                cv2.circle(img, (cX, cY), 5, (255, 255, 255), -1)
                cv2.putText(img, "centroid_"+str(i)+"_"+str(cX)+','+str(cY)    ,    (cX - 25, cY - 25)   ,cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 0), 2)
                print ('cX,cY',cX,cY)
                xyz=[]


                for jy in range (boundRect[0], boundRect[0]+boundRect[2]):
                    for ix in range(boundRect[1], boundRect[1]+boundRect[3]):
                        aux=(np.asarray((points_data['x'][ix,jy],points_data['y'][ix,jy],points_data['z'][ix,jy])))
                        if np.isnan(aux[0]) or np.isnan(aux[1]) or np.isnan(aux[2]):
                            'reject point'
                        else:
                            xyz.append(aux)

                xyz=np.asarray(xyz)
                cent=xyz.mean(axis=0)
                cents.append(cent)
                print (cent)
                points.append(xyz)
            else:
                print ('cent out of region... rejected')
                images.pop()
    sub_plt=0
    if plt_images:
        for image in images:

            sub_plt+=1
            ax = plt.subplot(5, 5, sub_plt )

            plt.imshow(image)
            plt.axis("off")

    cents=np.asarray(cents)
    #images.append(img)
    return(cents,np.asarray(points), images)


"""def gaze_point(x,y,z):

    ###Moves head to make center point of rgbd image th coordinates w.r.t.map
    ### To do: (Start from current pose  instead of always going to neutral first )
    
    
    
    head_pose = head.get_current_joint_values()
    head_pose[0]=0.0
    head_pose[1]=0.0
    head.set_joint_value_target(head_pose)
    head.go()
    
    trans , rot = listener.lookupTransform('/map', '/head_rgbd_sensor_gazebo_frame', rospy.Time(0)) #
    
    e =tf.transformations.euler_from_quaternion(rot)
    
    x_rob,y_rob,z_rob,th_rob= trans[0], trans[1] ,trans[2] ,  e[2]


    D_x=x_rob-x
    D_y=y_rob-y
    D_z=z_rob-z

    D_th= np.arctan2(D_y,D_x)
    #print('relative to robot',(D_x,D_y,np.rad2deg(D_th)))

    pan_correct= (- th_rob + D_th + np.pi) % (2*np.pi)

    if(pan_correct > np.pi):
        pan_correct=-2*np.pi+pan_correct
    if(pan_correct < -np.pi):
        pan_correct=2*np.pi+pan_correct

    if ((pan_correct) > .5 * np.pi):
        print ('Exorcist alert')
        pan_correct=.5*np.pi
    head_pose[0]=pan_correct
    tilt_correct=np.arctan2(D_z,np.linalg.norm((D_x,D_y)))

    head_pose [1]=-tilt_correct
    
    
    
    head.set_joint_value_target(head_pose)
    succ=head.go()
    return succ"""



def move_base(goal_x,goal_y,goal_yaw,time_out=10):
    nav_goal= NavigateActionGoal()
    nav_goal.goal.x= goal_x
    nav_goal.goal.y= goal_y
    nav_goal.goal.yaw=goal_yaw
    nav_goal.goal.timeout= time_out

    print (nav_goal)

    # send message to the action server
    navclient.send_goal(nav_goal.goal)


    # wait for the action server to complete the order
    navclient.wait_for_result(timeout=rospy.Duration(time_out))

    # print result of navigation
    action_state = navclient.get_state()
    return navclient.get_state()



def static_tf_publish(cents):
    ## Publish tfs of the centroids obtained w.r.t. head sensor frame and references them to map (static)
    #trans , rot = listener.lookupTransform('/map', '/head_rgbd_sensor_gazebo_frame', rospy.Time(0))
    trans, rot = tf_man.getTF(target_frame='head_rgbd_sensor_gazebo_frame')
    for  i ,cent  in enumerate(cents):
        x,y,z=cent
        if np.isnan(x) or np.isnan(y) or np.isnan(z):
            print('nan')
        else:
            #broadcaster.sendTransform((x,y,z),rot, rospy.Time.now(), 'Object'+str(i),"head_rgbd_sensor_link")
            tf_man.pub_tf(pos=[x,y,z], rot=rot, point_name=f'Object {i}', ref='head_rgbd_sensor_link')
            rospy.sleep(.2)
            #xyz_map,cent_quat= listener.lookupTransform('/map', 'Object'+str(i),rospy.Time(0))
            xyz_map,cent_quat = tf_man.getTF(target_frame=f'Object {i}')
            map_euler=tf.transformations.euler_from_quaternion(cent_quat)
            rospy.sleep(.2)
            #static_transformStamped = TransformStamped()
            

            ##FIXING TF TO MAP ( ODOM REALLY)    
            #tf_broadcaster1.sendTransform( (xyz[0],xyz[1],xyz[2]),tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time(0), "obj"+str(ind), "head_rgbd_sensor_link")
            """static_transformStamped.header.stamp = rospy.Time.now()
            static_transformStamped.header.frame_id = "map"
            
            static_transformStamped.transform.translation.x = float(xyz_map[0])
            static_transformStamped.transform.translation.y = float(xyz_map[1])
            static_transformStamped.transform.translation.z = float(xyz_map[2])
            #quat = tf.transformations.quaternion_from_euler(-euler[0],0,1.5)
            static_transformStamped.transform.rotation.x = 0#-quat[0]#trans.transform.rotation.x
            static_transformStamped.transform.rotation.y = 0#-quat[1]#trans.transform.rotation.y
            static_transformStamped.transform.rotation.z = 0#-quat[2]#trans.transform.rotation.z
            static_transformStamped.transform.rotation.w = 1#-quat[3]#trans.transform.rotation.w"""
            if xyz_map[2] > .7 and xyz_map[2] < .85:

                #static_transformStamped.child_frame_id = "Object_"+str(i)+"_Table_real_lab"
                #tf_static_broadcaster.sendTransform(static_transformStamped)
                tf_man.pub_static_tf(pos=xyz_map,point_name=f'Object_{i}_Table_real_lab')
                print (xyz_map[2])
            
            
            if xyz_map[2] > .4 and xyz_map[2] < .46:   #table 1 
                #static_transformStamped.child_frame_id = "Object_"+str(i)+"_Table_1"
                #tf_static_broadcaster.sendTransform(static_transformStamped)
                tf_man.pub_static_tf(pos=xyz_map,point_name=f'Object_{i}_Table_1')
                print (xyz_map[2])
            if  xyz_map[2] < .25:   #Floor
                #static_transformStamped.child_frame_id = "Object_"+str(i)+"_Floor"
                #tf_static_broadcaster.sendTransform(static_transformStamped)
                tf_man.pub_static_tf(pos=xyz_map,point_name=f'Object_{i}_Floor')
                print (xyz_map[2])
    return True
    

def move_d_to(target_distance=0.5,target_link='Floor_Object0'):
    ###Face towards Targetlink and get target distance close
    """try:
        #obj_tar,_ =  listener.lookupTransform('map',target_link,rospy.Time(0))
    except(tf.LookupException):
        print ('no  tf found')
        return False"""

    obj_tar,_=tf_man.getTF(target_frame='target_link')
    if not obj_tar:
        print ('no tf found ')
        return False
    #robot, quat_robot =  listener.lookupTransform('map','base_link',rospy.Time(0))
    #pose, quat =  listener.lookupTransform('base_link',target_link,rospy.Time(0))

    robot, quat_robot = tf_man.getTF(target_frame='base_link')
    pose, quat = tf_man.getTF(target_frame=target_link,ref_frame='base_link')
    D=np.asarray(obj_tar)-np.asarray(robot)
    d=D/np.linalg.norm(D)
    if target_distance==-1:
        new_pose=np.asarray(robot)
    else:
        new_pose=np.asarray(obj_tar)-target_distance*d
    
    #broadcaster.sendTransform(new_pose,(0,0,0,1), rospy.Time.now(), 'D_from_object','map')
    tf_man.pub_tf(pos=new_pose,point_name=f'D_from_object')
    wb_v=tf.transformations.euler_from_quaternion(quat_robot)

    succ=move_base( new_pose[0],new_pose[1],         np.arctan2(pose[1],pose[0])+wb_v[2])
    return succ  

        
import time
base_vel_pub = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=10)

def wait_for_face(timeout=10):
    
    rospy.sleep(0.3)
    
    start_time = rospy.get_time()
    strings=Strings()
    string_msg= String()
    string_msg.data='Anyone'
    while rospy.get_time() - start_time < timeout:
        img=rgbd.get_image()  
        req=RecognizeFaceRequest()
        print ('Got  image with shape',img.shape)
        req.Ids.ids.append(string_msg)
        img_msg=bridge.cv2_to_imgmsg(img)
        req.in_.image_msgs.append(img_msg)

        res= recognize_face(req)

        if res.Ids.ids[0].data == 'NO_FACE':
            print ('No face FOund Keep scanning')
        else:return res
    



"""def move_abs(vx,vy,vw, timeout=0.05):
    start_time = time.clock_gettime(0) 
    sec=time.clock_gettime(0)

    while sec- start_time < timeout: 
        sec=time.clock_gettime(0)
        twist = Twist()
        twist.linear.x = vx
        twist.linear.y = vy
        twist.angular.z = vw / 180.0 * np.pi  
        base_vel_pub.publish(twist)"""

##### Define state INITIAL #####

class Initial(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'],input_keys=['global_counter'])
        self.tries=0

        
    def execute(self,userdata):

        
        rospy.loginfo('STATE : INITIAL')
        print('robot neutral pose')

        print('Try',self.tries,'of 5 attempts') 
        self.tries+=1
        if self.tries==3:
            return 'tries'
        
        clear_octo_client()
        
        #scene.remove_world_object()
        #Takeshi neutral
        arm.set_named_target('go')
        arm.go()
        head.set_named_target('neutral')
        succ=head.go() 
        
        #succ = True        
        if succ:
            return 'succ'
        else:
            return 'failed'

class Wait_push_hand(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'],input_keys=['global_counter'])
        self.tries=0

        
    def execute(self,userdata):
        self.tries+=1
        if self.tries==3:return 'tries'
        rospy.loginfo('STATE : WAIT_PUSH_HAND')
        takeshi_talk_pub.publish(string_to_Voice('Gently push my hand to begin'))

        succ=wait_for_push_hand(40) 
        
        #succ = True        
        if succ:
            return 'succ'
        else:
            return 'failed'



class Scan_face(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'])
        self.tries=0
    def execute(self,userdata):

        rospy.loginfo('State : SCAN_FACE')
        self.tries+=1        
        
        if self.tries==1:
            head.set_named_target('neutral')
            head.go() 
        if self.tries==2:
            hv= head.get_current_joint_values()
            hv[0]= -0.4  ##mav val ?
            hv[1]= 0.0
            head.go(hv) 
        if self.tries==3:
            hv= head.get_current_joint_values()
            hv[0]= 0.6
            hv[1]= 0.0
            head.go(hv) 
        if self.tries>=4:
            self.tries=0
            return'tries'
        
        #img=rgbd.get_image()  
        #req=RecognizeFaceRequest()
        #print ('Got  image with shape',img.shape)
        #strings=Strings()
        #string_msg= String()
        #string_msg.data='Anyone'
        #req.Ids.ids.append(string_msg)
        #img_msg=bridge.cv2_to_imgmsg(img)
        #req.in_.image_msgs.append(img_msg)
        #res= recognize_face(req)
        res=wait_for_face()##default 10 secs
        
        print('Checking for faces')
        if res== None:
            return 'failed'
        if res != None:
            print('RESPONSE',res.Ids.ids      )
            if res.Ids.ids[0].data == 'NO_FACE':
                print ('No face Found Keep scanning')
                return 'failed'
            else:
                print ('A face was found.')
                #trans,rot=listener.lookupTransform('/map', '/head_rgbd_sensor_link', rospy.Time(0))  
                trans, rot = tf_man.getTF(target_frame='head_rgbd_sensor_link')
                print (trans , rot)
                trans= np.zeros(3)
                trans[2]+=res.Ds.data[0]

                #broadcaster.sendTransform( trans,(0,0,0,1),rospy.Time.now(), 'Face','head_rgbd_sensor_link')            #res.Ids.ids[0].data
                tf_man.pub_static_tf(pos=trans,point_name='Face', ref='head_rgbd_sensor_link')

                #advice: change 'Face' reference frame
                #tf_man.change_ref_frame_tf(point_name='Face', new_frame='map')
                rospy.sleep(0.25)
                return 0
                return 'succ'
                    
        



"""print('Looking for FACE (DLIB) try num ',self.tries)
        img=rgbd.get_image()        
        print (img.shape)
        
   """
        
class Goto_face(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'])
        self.tries=0
    def execute(self,userdata):


        rospy.loginfo('State : GOING TO FACE PUMAS NAV AND MAP')
        #goal_pose, quat=listener.lookupTransform( 'map','Face', rospy.Time(0))
        goal_pose, quat = tf_man.getTF(target_frame='Face', ref_frame='map')
        print(goal_pose)
        return 0
        ###move_base(goal_pose[0],goal_pose[1],0,10  )   #X Y YAW AND TIMEOUT
        move_d_to(0.7,'Face')


        

        


        
        return 'succ'
        
        

        self.tries+=1        






def init(node_name):
    global listener, broadcaster, tfBuffer, tf_static_broadcaster, scene, rgbd  , head,whole_body,arm,gripper  ,goal,navclient,clear_octo_client ,  detect_waving_client, class_names , bridge , base_vel_pub,takeshi_talk_pub, order, navclient, recognize_face, pub_potfields_goal, tf_man
    rospy.init_node(node_name)
    head = moveit_commander.MoveGroupCommander('head')
    gripper =  moveit_commander.MoveGroupCommander('gripper')
    #whole_body=moveit_commander.MoveGroupCommander('whole_body')
    arm =  moveit_commander.MoveGroupCommander('arm')
    #listener = tf.TransformListener()
    #broadcaster = tf.TransformBroadcaster()
    #navclient = actionlib.SimpleActionClient('/move_base/move', MoveBaseAction)
    #tfBuffer = tf2_ros.Buffer()
    #tf_static_broadcaster = tf2_ros.StaticTransformBroadcaster()
    #whole_body.set_workspace([-6.0, -6.0, 6.0, 6.0]) 
    scene = moveit_commander.PlanningSceneInterface()
    rgbd = RGB()
    goal = MoveBaseGoal()
    
    #############################################################################
    #navclient = actionlib.SimpleActionClient('/move_base/move', MoveBaseAction)#
    ###FANFARRIAS Y REDOBLES PUMASNAVIGATION ####################################
    #############################################################################

    pub_potfields_goal = rospy.Publisher("/clicked_point",PointStamped,queue_size=10)

    navclient = actionlib.SimpleActionClient('/navigate', NavigateAction)   ### PUMAS NAV ACTION LIB
    
    clear_octo_client = rospy.ServiceProxy('/clear_octomap', Empty)
    bridge = CvBridge()
    #class_names=['002masterchefcan', '003crackerbox', '004sugarbox', '005tomatosoupcan', '006mustardbottle', '007tunafishcan', '008puddingbox', '009gelatinbox', '010pottedmeatcan', '011banana', '012strawberry', '013apple', '014lemon', '015peach', '016pear', '017orange', '018plum', '019pitcherbase', '021bleachcleanser', '022windexbottle', '024bowl', '025mug', '027skillet', '028skilletlid', '029plate', '030fork', '031spoon', '032knife', '033spatula', '035powerdrill', '036woodblock', '037scissors', '038padlock', '040largemarker', '042adjustablewrench', '043phillipsscrewdriver', '044flatscrewdriver', '048hammer', '050mediumclamp', '051largeclamp', '052extralargeclamp', '053minisoccerball', '054softball', '055baseball', '056tennisball', '057racquetball', '058golfball', '059chain', '061foambrick', '062dice', '063-amarbles', '063-bmarbles', '065-acups', '065-bcups', '065-ccups', '065-dcups', '065-ecups', '065-fcups', '065-gcups', '065-hcups', '065-icups', '065-jcups', '070-acoloredwoodblocks', '070-bcoloredwoodblocks', '071nineholepegtest', '072-atoyairplane', '073-alegoduplo', '073-blegoduplo', '073-clegoduplo', '073-dlegoduplo', '073-elegoduplo', '073-flegoduplo', '073-glegoduplo']
    #classify_client = rospy.ServiceProxy('/classify', Classify)
    print ('Waiting for face recog service')
    rospy.wait_for_service('recognize_face')
    recognize_face = rospy.ServiceProxy('recognize_face', RecognizeFace)    
    train_new_face = rospy.ServiceProxy('new_face', RecognizeFace)    
    base_vel_pub = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=10)
    takeshi_talk_pub = rospy.Publisher('/talk_request', Voice, queue_size=10)
    tf_man = TF_MANAGER()
    grip = GRIPPER()

#Entry point    
if __name__== '__main__':
    print("Takeshi STATE MACHINE...")
    init("takeshi_smach")
    sm = smach.StateMachine(outcomes = ['END'])     #State machine, final state "END"
    
    #sm.userdata.clear = False
    sis = smach_ros.IntrospectionServer('SMACH_VIEW_SERVER', sm, '/SM_ROOT')
    sis.start()


    with sm:
        #State machine for Restaurant
        smach.StateMachine.add("INITIAL",           Initial(),          transitions = {'failed':'INITIAL',          'succ':'WAIT_PUSH_HAND',           'tries':'END'}) 
        smach.StateMachine.add("WAIT_PUSH_HAND",           Wait_push_hand(),          transitions = {'failed':'WAIT_PUSH_HAND',          'succ':'SCAN_FACE',           'tries':'END'}) 
        smach.StateMachine.add("SCAN_FACE",   Scan_face(),  transitions = {'failed':'SCAN_FACE',  'succ':'GOTO_FACE',    'tries':'INITIAL'}) 
        smach.StateMachine.add("GOTO_FACE",   Goto_face(),  transitions = {'failed':'GOTO_FACE',  'succ':'INITIAL',    'tries':'INITIAL'}) 
    outcome = sm.execute()

 
    

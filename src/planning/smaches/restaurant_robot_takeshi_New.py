#!/usr/bin/env python3
from std_srvs.srv import Empty, Trigger, TriggerRequest
import smach
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, Point , Quaternion
from actionlib_msgs.msg import GoalStatus
from tmc_msgs.msg import Voice
import moveit_commander
import moveit_msgs.msg
import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from object_classification.srv import *
from hsrb_interface import Robot
import cv2  # New
import rospy # New
#import face_recognition #New

from cv_bridge import CvBridge, CvBridgeError

from utils_notebooks import *
from utils_takeshi import *
from utils_srv import *   #New
from sensor_msgs.msg import Image , LaserScan , PointCloud2



from hri_msgs.msg import RecognizedSpeech
from std_msgs.msg import Bool

from hsrb_interface import Robot
import traceback
protoFile = "/home/takeshi/openpose/models/pose/body_25/pose_deploy.prototxt"
weightsFile = "/home/takeshi/openpose/models/pose/body_25/pose_iter_584000.caffemodel"

net = cv2.dnn.readNetFromCaffe(protoFile, weightsFile)

robot = Robot()
whole_body = robot.get("whole_body")
omni_base = robot.get("omni_base") #Standard initialisation (Toyota)
gripper = robot.get('gripper')
collision = robot.get('global_collision_world')

########## Functions for takeshi states ##########
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

def open_gripper():
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
    succ=gripper.go()
def correct_points(low=.27,high=1000):

    #Corrects point clouds "perspective" i.e. Reference frame head is changed to reference frame map
    data = rospy.wait_for_message('/hsrb/head_rgbd_sensor/depth_registered/rectified_points', PointCloud2)
    np_data=ros_numpy.numpify(data)
    trans,rot=listener.lookupTransform('/map', '/head_rgbd_sensor_gazebo_frame', rospy.Time(0))

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


def gaze_point(x,y,z):

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
    return succ


# def move_base(goal_x,goal_y,goal_yaw,time_out=10):

#     #using nav client and toyota navigation go to x,y,yaw
#     #To Do: PUMAS NAVIGATION
#     pose = PoseStamped()
#     pose.header.stamp = rospy.Time(0)
#     pose.header.frame_id = "map"
#     pose.pose.position = Point(goal_x, goal_y, 0)
#     quat = tf.transformations.quaternion_from_euler(0, 0, goal_yaw)
#     pose.pose.orientation = Quaternion(*quat)


#     # create a MOVE BASE GOAL
#     goal = MoveBaseGoal()
#     goal.target_pose = pose

#     # send message to the action server
#     navclient.send_goal(goal)

#     # wait for the action server to complete the order
#     navclient.wait_for_result(timeout=rospy.Duration(time_out))

#     # print result of navigation
#     action_state = navclient.get_state()
#     print(action_state)
#     return navclient.get_state()

def move_base(goal_x,goal_y,goal_yaw,time_out=10):

    #using nav client and toyota navigation go to x,y,yaw
    #To Do: PUMAS NAVIGATION

    state = omni_base.go_rel(goal_x, goal_y, goal_yaw, timeout=100)
    print(state)

    return state

def static_tf_publish(cents):
    ## Publish tfs of the centroids obtained w.r.t. head sensor frame and references them to map (static)
    trans , rot = listener.lookupTransform('/map', '/head_rgbd_sensor_gazebo_frame', rospy.Time(0))
    for  i ,cent  in enumerate(cents):
        x,y,z=cent
        if np.isnan(x) or np.isnan(y) or np.isnan(z):
            print('nan')
        else:
            broadcaster.sendTransform((x,y,z),rot, rospy.Time.now(), 'Object'+str(i),"head_rgbd_sensor_link")
            rospy.sleep(.2)
            xyz_map,cent_quat= listener.lookupTransform('/map', 'Object'+str(i),rospy.Time(0))
            map_euler=tf.transformations.euler_from_quaternion(cent_quat)
            rospy.sleep(.2)
            static_transformStamped = TransformStamped()


            ##FIXING TF TO MAP ( ODOM REALLY)
            #tf_broadcaster1.sendTransform( (xyz[0],xyz[1],xyz[2]),tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time(0), "obj"+str(ind), "head_rgbd_sensor_link")
            static_transformStamped.header.stamp = rospy.Time.now()
            static_transformStamped.header.frame_id = "map"

            static_transformStamped.transform.translation.x = float(xyz_map[0])
            static_transformStamped.transform.translation.y = float(xyz_map[1])
            static_transformStamped.transform.translation.z = float(xyz_map[2])
            #quat = tf.transformations.quaternion_from_euler(-euler[0],0,1.5)
            static_transformStamped.transform.rotation.x = 0#-quat[0]#trans.transform.rotation.x
            static_transformStamped.transform.rotation.y = 0#-quat[1]#trans.transform.rotation.y
            static_transformStamped.transform.rotation.z = 0#-quat[2]#trans.transform.rotation.z
            static_transformStamped.transform.rotation.w = 1#-quat[3]#trans.transform.rotation.w
            if xyz_map[2] > .7 and xyz_map[2] < .85:
                static_transformStamped.child_frame_id = "Object_"+str(i)+"_Table_real_lab"
                tf_static_broadcaster.sendTransform(static_transformStamped)
                print (xyz_map[2])


            if xyz_map[2] > .4 and xyz_map[2] < .46:   #table 1
                static_transformStamped.child_frame_id = "Object_"+str(i)+"_Table_1"
                tf_static_broadcaster.sendTransform(static_transformStamped)
                print (xyz_map[2])
            if  xyz_map[2] < .25:   #Floor
                static_transformStamped.child_frame_id = "Object_"+str(i)+"_Floor"
                tf_static_broadcaster.sendTransform(static_transformStamped)
                print (xyz_map[2])
    return True


def move_d_to(target_distance=0.5,target_link='Floor_Object0'):
    ###Face towards Targetlink and get target distance close
    try:
        obj_tar,_ =  listener.lookupTransform('map',target_link,rospy.Time(0))
    except(tf.LookupException):
        print ('no  tf found')
        return False

    robot, _ =  listener.lookupTransform('map','base_link',rospy.Time(0))
    pose, quat =  listener.lookupTransform('base_link',target_link,rospy.Time(0))

    D=np.asarray(obj_tar)-np.asarray(robot)
    d=D/np.linalg.norm(D)
    if target_distance==-1:
        new_pose=np.asarray(robot)
    else:
        new_pose=np.asarray(obj_tar)-target_distance*d

    broadcaster.sendTransform(new_pose,(0,0,0,1), rospy.Time.now(), 'D_from_object','map')
    wb_v= whole_body.get_current_joint_values()

    arm.set_named_target('go')
    arm.go()


    succ=move_base( new_pose[0],new_pose[1],         np.arctan2(pose[1],pose[0])+wb_v[2])
    return succ

    #############################################################################
def predict_waving(frame):


    # Specify the input image dimensions
    inHeight = frame.shape[0]
    inWidth = frame.shape[1]


    # Prepare the frame to be fed to the network
    inpBlob = cv2.dnn.blobFromImage(frame, 1.0 / 255, (inWidth, inHeight), (0, 0, 0), swapRB=False, crop=False)

    # Set the prepared object as the input blob of the network
    net.setInput(inpBlob)

    output = net.forward()

    H = output.shape[2]
    W = output.shape[3]
    threshold=0.1
    # Empty list to store the detected keypoints
    points = []
    invalid_joints = []

    for i in range(12):
        # confidence map of corresponding body's part.
        probMap = output[0, i, :, :]

        # Find global maxima of the probMap.
        _, prob,_, point = cv2.minMaxLoc(probMap)
        #print (prob,point)

        # Scale the point to fit on the original image
        x = (inWidth * point[0]) / W
        y = (inHeight * point[1]) / H

        
        # Add the point to the list if the probability is greater than the threshold
        points.append((int(x), int(y)))
    else :
        points.append(None)
        invalid_joints.append(i)
        
        print(points)
        
        # Logic

        joints1 = [2,3,5,6] # elbows and shoulders
        joints2 = [3,4,6,7] #

    if any(x in joints1 for x in invalid_joints): #if doesn't find the elbow or shoulder go out
        print("Out")
        return False , (0,0)
    else:
        if points[3][1]<points[2][1] or points[6][1]<points[5][1]: # ask about elbow over shoulder
            print('elbow up') 
            return True, points[0][:]
        else:
            print('elbow down, ask for wrist')
            if any(x in joints2 for x in invalid_joints): # if doesn't find the wrist go out 
                print("Out")
                return False, points[0][:]
            else:
                if points[4][1]<points[3][1] or points[7][1]<points[6][1]: # ask about wrist over elbow
                    print('Hand up')
                    return True, points[0][:]
                else:
                    print('Hand down, go Out')
                    return False ,  points[0][:]
           
############################################


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
        if self.tries==6:
            return 'tries'

        clear_octo_client()
        close_gripper()
        
        scene.remove_world_object()
        #Takeshi neutral
        arm.set_named_target('go')
        arm.go()
        head.set_named_target('neutral')
        succ = head.go()
        if succ:
            return 'succ'
        else:
            return 'failed'

class Mapping(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'])
        self.tries=0
    def execute(self,userdata):
        bar_location = [0,0,0]
        
        rospy.loginfo('State : MAPPING')
        head.set_named_target('neutral')
        head.go()
        print('360 Deg turn to Map')

        rospy.logerr("-" * 300)
        #omni_base.go_rel(0,0,np.pi)
        rospy.sleep(0.5)

        #omni_base.go_rel(0,0,np.pi)
        succ = True
        if succ:
            print('Takeshi Mapped')
            return 'succ'
        else:
            print('Panning for Map incomplete')
            return 'failed'


        self.tries+=1
        if self.tries==3:
            self.tries=0
            return'tries'
        else:
            return 'failed'


class Scan_restaurant(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'])
        self.tries=0
    def execute(self,userdata):
        global person_goal


        #Initializing person_goal
        person_goal = [1,1,1.70]

        rospy.loginfo('State : SCAN_RESTAURANT')
        head.set_named_target('neutral')
        head.go()

        WaveReq = False

        print('Looking for Waving Person - IN FRONT')
        for i in range(5):
            print('Pose: ',i)
            head_goal = head.get_current_joint_values()
            ang = (np.pi/4)*(i-2)
            print(ang)
            head_goal[0] = ang
            head_goal[1] = np.deg2rad(0)
            head.set_joint_value_target(head_goal)
            head.go()
            rospy.sleep(0.5)
        
           # Read image and PoinClouds topics
            data = rospy.wait_for_message("/hsrb/head_rgbd_sensor/rgb/image_raw",Image) ### FOR DEBUGGING: WHEN USING ROBOT PLEASE CHANGE THIS TOPIC ACCORDINGLY
            cv2_img = bridge.imgmsg_to_cv2(data)#, "bgr8")
            print(cv2_img.shape)
            data = rospy.wait_for_message('/hsrb/head_rgbd_sensor/depth_registered/rectified_points', PointCloud2)
            points=ros_numpy.numpify(data)
            red , face_coords =predict_waving(cv2_img)
            print (points.shape)
            xyz_wrt_robot= points[ face_coords[1],face_coords[0] ]
            WaveReq = red
            print('WaveReq=', red)
            


            if WaveReq == True:
                
                if 'nan' in [xyz_wrt_robot[0], xyz_wrt_robot[1], xyz_wrt_robot[2]]:
                    succ = False
                else:
                    print('Takeshi found a Waving Person')
                    print('Coordenadasss', xyz_wrt_robot[0],xyz_wrt_robot[1],xyz_wrt_robot[2] )
                    
                    #coordenadas  el robot respecto al mapa
                    trans , rot = listener.lookupTransform('/map', '/head_rgbd_sensor_link', rospy.Time(0))
                    # Coordenadas rostro humano detectado respecto al robot
                    broadcaster.sendTransform((xyz_wrt_robot[0],xyz_wrt_robot[1],xyz_wrt_robot[2] - 1.5),rot, rospy.Time.now(), 'Human_waving',"head_rgbd_sensor_link")
                    rospy.sleep(0.3)
                    # Coordenadas rostro humano detectado respecto al mapa trans_map
                    trans_map , rot_map = listener.lookupTransform('/map', 'Human_waving', rospy.Time(0))
                    succ = True
                    person_goal = trans_map
                    print('Coordinates of Person Saved', trans_map)
                    break
                   
            else: print('Clear space, Takeshi did not find a Waving Person in iteration', i)


        if WaveReq == False:
            print('Looking for Waving Person - BEHIND')
            move_base(0,0,np.pi)
            print('Moving Base ang = ',np.pi)
            rospy.sleep(0.5)

            for i in range(5):
                print('Pose: ',i)
                head_goal = head.get_current_joint_values()
                ang = (np.pi/4)*(i-2)
                print(ang)
                head_goal[0] = ang
                head_goal[1] = np.deg2rad(0)
                head.set_joint_value_target(head_goal)
                head.go()
                rospy.sleep(0.5)

                #Request LoreService

                
                data = rospy.wait_for_message("/hsrb/head_rgbd_sensor/rgb/image_raw",Image) ### FOR DEBUGGING: WHEN USING ROBOT PLEASE CHANGE THIS TOPIC ACCORDINGLY
                cv2_img = bridge.imgmsg_to_cv2(data)#, "bgr8")
                print(cv2_img.shape)
                red , face_coords =predict_waving(cv2_img)
                data = rospy.wait_for_message('/hsrb/head_rgbd_sensor/depth_registered/rectified_points', PointCloud2)
                points=ros_numpy.numpify(data)
                xyz_wrt_robot= points[ face_coords[1],face_coords[0] ]
                WaveReq = red
                print('WaveReq=', red)
                


                if WaveReq == True:
                    if 'nan' in [xyz_wrt_robot[0], xyz_wrt_robot[1], xyz_wrt_robot[2]]:
                        succ = False
                    else:
                        print('Takeshi found a Waving Person')
                        print(xyz_wrt_robot[0],xyz_wrt_robot[1],xyz_wrt_robot[2] )
                        
                        #coordenadas  el robot respecto al mapa
                        trans , rot = listener.lookupTransform('/map', '/head_rgbd_sensor_link', rospy.Time(0))
                        # Coordenadas rostro humano detectado respecto al robot
                        broadcaster.sendTransform((xyz_wrt_robot[0],xyz_wrt_robot[1],xyz_wrt_robot[2] - 1.5),rot, rospy.Time.now(), 'Human_waving',"head_rgbd_sensor_link")
                        rospy.sleep(0.3)
                        # Coordenadas rostro humano detectado respecto al mapa trans_map
                        trans_map , rot_map = listener.lookupTransform('/map', 'Human_waving', rospy.Time(0))
                        succ = True
                        person_goal = trans_map
                        print('Coordinates of Person Saved',trans_map)
                        break
                        
                else: 
                    print('Clear space, Takeshi did not find a Waving Person in iteration', i)
                    succ = False
        
        move_base(0,0,0)
        head.set_named_target('neutral')
        head.go()
        rospy.sleep(1)

        #succ = True
        #Comment for Robot execution. Simulation does not have waving persons
        if succ:
            return 'succ'
        else:
            return 'failed'


        self.tries+=1
        if self.tries==3:
            self.tries=0
            return'tries'
        else:
            return 'failed' 


class Goto_table(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries','end'])
        self.tries=0
    def execute(self,userdata):
        self.tries+=1

        global cents, rot, trans, person_goal

        rospy.loginfo('State : GOTO_TABLE')

        #LOCATION GIVEN BY WAVE DETECTOR USING GOALS FOR EASE
        #Remove with Lore Integration
        goal_x = 0.3 + 0.051*self.tries
        goal_y = 1.2
        goal_yaw = 1.57
        goal_xyz=np.asarray((goal_x,goal_y,goal_yaw))

        succ = move_base(person_goal[0],person_goal[1],0)
        print('moving to (',person_goal[0],person_goal[1],0,')')
        xyz=whole_body.get_current_joint_values()[:3]
        #succ = True
        print ('Goal is table',goal_xyz,'current',xyz)
        print ('Distance is ' ,np.linalg.norm(xyz-goal_xyz))
        if (np.linalg.norm(xyz-goal_xyz)  < .3):
            rospy.loginfo("Navigation Close Enough.")
            return 'succ'
        if succ:
            return 'succ'

        if self.tries==5:
            self.tries=0
            return'tries'

        else:
            return'failed'

class Hri_take_order(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'])
        self.tries=0
    def execute(self,userdata):
        self.tries+=1
        global cents, rot, trans, order, person_goal
        rospy.loginfo('State : HRI_TAKE_ORDER')

        #Check
        #gaze_point(person_goal[0],person_goal[1],person_goal[2])

#////////////////////////////////////////////TAKE ORDER///////////////////////////////////
        voice_message=Voice()
        voice_message.sentence = 'Hi, can I take your order'
        voice_message.queueing = False
        voice_message.language = 1
        voice_message.interrupting = False
        #print('--------------------------PUB------------------------')

        takeshi_talk_pub.publish(voice_message)
        rospy.sleep(1.5)

        print('--------------------------MIC TRUE-------------------------')
        mic_sphinx.publish(True) #<------------------------------------------------------------/////////////////////////////////
        rospy.sleep(.2)

        print ("-------------------------------")
        print ("Please Speak now")
        data = rospy.wait_for_message('/recognizedSpeech', RecognizedSpeech)

        d = str(data)
        wordlist = d.split()
        wordlist.pop()
        wordlist.remove('hypothesis:')
        wordlist.remove('-')
        wordlist.remove('confidences:')
        order = " "
        for word in wordlist:
            if word in ["apple", "orange", "banana", "strawberry", "lemon", "peach", "pear", "plum", "cookies", "pudding", "gelatin", "meat can", "coffee", "soup", "tuna can", "sugar", "mustard", "chips"]:
                order =  word
                print('Order: ', order)
                succ = True
            else:
                succ = False
        print ("-------------------------------")

        rospy.sleep(2)
        mic_sphinx.publish(False) #<---------------------------------------------------------////////////////////////////////////
        print('-------------------------MIC FALSE----------------------------------------')
#/////////////////////////////////////////////////////////////////////////////////

        if succ:
            return 'succ'

        if self.tries==5:
            self.tries=0
            return'tries'

        else:
            return'failed'

class Hri_confirm_order(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'])
        self.tries=0
    def execute(self,userdata):
        self.tries+=1
        global cents, rot, trans, order
        rospy.loginfo('State : HRI_CONFIRM_ORDER')

        #gaze_point(person_goal[0],person_goal[1],person_goal[2])

#////////////////////////////////////////CONFIRM ORDER////////////////////////////////////
        rospy.sleep(2)
        voice_message2=Voice()
        voice_message2.sentence = 'Do you want a '+ order
        voice_message2.queueing = False
        voice_message2.language = 1
        voice_message2.interrupting = False

        takeshi_talk_pub.publish(voice_message2)
        rospy.sleep(0.3)


        print('Please confirm your order. Do you want a '+ order)
        rospy.sleep(1)

        print ("-------------------------------")
        print ("Please Speak now")
        print('-------------------------MIC TRUE----------------------------------------')
        mic_sphinx.publish(True) #<----------------------------------------------------------///////////////////////////////
        rospy.sleep(0.2)

        data_conf = rospy.wait_for_message('/recognizedSpeech', RecognizedSpeech)

        d1 = str(data_conf)
        wordlist = d1.split()
        wordlist.pop()
        wordlist.remove('hypothesis:')
        wordlist.remove('-')
        wordlist.remove('confidences:')
        print(wordlist)

        confirmation = " "
        for word in wordlist:
            if word in ["yes", "no", "'yes'", "'no'"]:
                print('I heard ',word)
                confirmation = word

                if (confirmation == "yes"):
                    print('Confirmation, I heard ',confirmation)
                    succ = True
                elif (confirmation == "'yes'"):
                    print('Confirmation, Mistaken Order, I heard ',confirmation)
                    succ = True
                elif (confirmation == "'no'"):
                    print('Confirmation, Mistaken Order, I heard ',confirmation)
                    succ = False
                elif (confirmation == "no"):
                    print('Confirmation, Mistaken Order, I heard ',confirmation)
                    succ = False
            else:
                succ = False

        print ("-------------------------------")

        mic_sphinx.publish(False) #<--------------------------------------------------------/////////////////////////////////
        #rospy.sleep(1)
        print('-------------------------MIC FALSE----------------------------------------')
#///////////////////////////////////////////////////////////////////////////////////////
        """wordlist = data_conf.hypothesis
        confirmation = " "
        for word in wordlist:
            print('I heard ',word)
            confirmation = word
            if (confirmation == 'yes'):
                print('Confirmation, I heard ',confirmation)
                succ = True
            else: #confirmation == 'no':
                print('Confirmation, Mistaken Order, I heard ',confirmation)
                succ = False
        print ("-------------------------------")"""

        if succ:
            return 'succ'

        if self.tries==5:
            self.tries=0
            return'tries'

        else:
            return'failed'

class Goto_bar(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'])
        self.tries=0
    def execute(self,userdata):
        self.tries+=1

        global cents, rot, trans, bar_location
        rospy.loginfo('State : GOTO_BAR')

        goal_x = 1.98
        goal_y = 0.128
        goal_yaw = -1.57
        goal_xyz=np.asarray((goal_x,goal_y,goal_yaw))

        succ=move_base(goal_x, goal_y, goal_yaw)
        xyz=whole_body.get_current_joint_values()[:3]
        #succ = True
        print ('Goal is bar',goal_xyz,'current',xyz)
        print ('Distance is ' ,np.linalg.norm(xyz-goal_xyz))
        if (np.linalg.norm(xyz-goal_xyz)  < .3):
            rospy.loginfo("Navigation Close Enough.")
            return 'succ'
        if succ:
            return 'succ'

        if self.tries==5:
            self.tries=0
            return'tries'

        else:
            return'failed'

class Scan_bar(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'])
        self.tries=0
    def execute(self,userdata):
        self.tries+=1
        if self.tries==3:
            self.tries=0
            return'tries'
        global cents, rot, trans
        rospy.loginfo('State : SCAN_BAR')

        #Table Z


        #TAMAGAWA Classification

        print('ObjectTF found')
        succ = True
        if succ:
            return 'succ'
        else:
            return 'failed'


        if self.tries==5:
            self.tries=0
            return'tries'

        else:
            return'failed'


class Pre_grasp_bar(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'])
        self.tries=0
    def execute(self,userdata):
        rospy.loginfo('State : PRE_GRASP_BAR')
        print ("self.tries",self.tries)
        #target_tf= 'Object_1_Table_1'

        #move_d_to(0.8,target_tf)
        head.set_named_target('neutral')
        head.go()
        arm.set_named_target('neutral')
        arm.go()


        arm_grasp_table=[0.31349380130577407, -1.671584191489468,-0.02774372779356371,0.0,0.22362492457833927,0.0]
        succ = arm.go(arm_grasp_table)

        #TAMAGAWA grasp

        if self.tries==3:
            self.tries=0
            return'tries'
        if succ:
            return 'succ'
        else:
            return 'failed'

class Grasp_bar(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'])
        self.tries=0
    def execute(self,userdata):
        rospy.loginfo('State : GRASP_BAR')

        open_gripper()

        print('Taking Orden from Bar')
        rospy.sleep(1)
        goal_x = 2.0
        goal_y = 0.108
        goal_yaw = -1.57
        goal_xyz=np.asarray((goal_x,goal_y,goal_yaw))

        move_base(goal_x, goal_y, goal_yaw)

        close_gripper()

        arm.set_named_target('go')
        arm.go()
        head.set_named_target('neutral')
        succ = head.go()

        #TAMAGAWA graspeo

        self.tries+=1
        if self.tries==3:
            self.tries=0
            return'tries'
        if succ:
            return 'succ'
        else:
            return 'failed'


class Goto_del_goal(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'])
        self.tries=0
    def execute(self,userdata):
        global person_goal
        self.tries+=1
        if self.tries==4:
            self.tries=0
            return'tries'
        rospy.loginfo('State : GOTO_DEL_GOAL')


        goal_x = 0.3 + 0.051*self.tries
        goal_y = 1.2
        goal_yaw = 1.57
        goal_xyz=np.asarray((goal_x,goal_y,goal_yaw))

        move_base(goal_x, goal_y, goal_yaw)
        xyz=whole_body.get_current_joint_values()[:3]

        print ('Goal is bar',goal_xyz,'current',xyz)
        print ('Distance is ' ,np.linalg.norm(xyz-goal_xyz))
        if (np.linalg.norm(xyz-goal_xyz)  < .3):
            rospy.loginfo("Navigation Close Enough.")

        head.set_named_target('neutral')
        succ = head.go()

        if succ:
            return 'succ'
        else:
            return'failed'

class Delivery(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'])
        self.tries=0
    def execute(self,userdata):
        self.tries+=1
        if self.tries==4:
            self.tries=0
            return'tries'
        rospy.loginfo('State : DELIVERY')

        #gaze_point(person_goal[0],person_goal[1],person_goal[2])

#/////////////////////////////HERE IS YOUR ORDER/////////////////////////
        voice_message3=Voice()
        voice_message3.sentence = 'Here is your order'
        voice_message3.queueing = False
        voice_message3.language = 1
        voice_message3.interrupting = False

        takeshi_talk_pub.publish(voice_message3)
        rospy.sleep(.2)
#/////////////////////////////////////////////////////////////////////////////
        arm_deliver_table=[0.31349380130577407, -1.671584191489468,-0.02774372779356371,0.0,0.22362492457833927,0.0]
        arm.go(arm_deliver_table)
        open_gripper()

        print('Order Delivered')

        head.set_named_target('neutral')
        head.go()
        arm.set_named_target('neutral')
        succ = arm.go()

        if succ:
            return 'succ'
        else:
            return'failed'

def init(node_name):
    global listener, broadcaster, tfBuffer, tf_static_broadcaster, scene, rgbd  , head,whole_body,arm,gripper  ,goal,navclient,clear_octo_client, mic_sphinx
    global classify_client , detect_waving_client, class_names , bridge , base_vel_pub,takeshi_talk_pub, order, person_goal
    #rospy.init_node(node_name)
    head = moveit_commander.MoveGroupCommander('head', wait_for_servers=300)
    gripper =  moveit_commander.MoveGroupCommander('gripper', wait_for_servers=300)
    whole_body=moveit_commander.MoveGroupCommander('whole_body', wait_for_servers=300)
    arm =  moveit_commander.MoveGroupCommander('arm', wait_for_servers=300)
    listener = tf.TransformListener()
    broadcaster = tf.TransformBroadcaster()
    tfBuffer = tf2_ros.Buffer()
    tf_static_broadcaster = tf2_ros.StaticTransformBroadcaster()
    whole_body.set_workspace([-6.0, -6.0, 6.0, 6.0])
    scene = moveit_commander.PlanningSceneInterface()
    rgbd = RGBD()
    goal = MoveBaseGoal()
    navclient = actionlib.SimpleActionClient('/move_base/move', MoveBaseAction)
    clear_octo_client = rospy.ServiceProxy('/clear_octomap', Empty)
    bridge = CvBridge()
    class_names=['002masterchefcan', '003crackerbox', '004sugarbox', '005tomatosoupcan', '006mustardbottle', '007tunafishcan', '008puddingbox', '009gelatinbox', '010pottedmeatcan', '011banana', '012strawberry', '013apple', '014lemon', '015peach', '016pear', '017orange', '018plum', '019pitcherbase', '021bleachcleanser', '022windexbottle', '024bowl', '025mug', '027skillet', '028skilletlid', '029plate', '030fork', '031spoon', '032knife', '033spatula', '035powerdrill', '036woodblock', '037scissors', '038padlock', '040largemarker', '042adjustablewrench', '043phillipsscrewdriver', '044flatscrewdriver', '048hammer', '050mediumclamp', '051largeclamp', '052extralargeclamp', '053minisoccerball', '054softball', '055baseball', '056tennisball', '057racquetball', '058golfball', '059chain', '061foambrick', '062dice', '063-amarbles', '063-bmarbles', '065-acups', '065-bcups', '065-ccups', '065-dcups', '065-ecups', '065-fcups', '065-gcups', '065-hcups', '065-icups', '065-jcups', '070-acoloredwoodblocks', '070-bcoloredwoodblocks', '071nineholepegtest', '072-atoyairplane', '073-alegoduplo', '073-blegoduplo', '073-clegoduplo', '073-dlegoduplo', '073-elegoduplo', '073-flegoduplo', '073-glegoduplo']
    classify_client = rospy.ServiceProxy('/classify', Classify)
    detect_waving_client = rospy.ServiceProxy('/detect_waving', Trigger) #New
    base_vel_pub = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=10)
    takeshi_talk_pub = rospy.Publisher('/talk_request', Voice, queue_size=10)
    mic_sphinx = rospy.Publisher('/pocketsphinx/mic', Bool, queue_size=10)

#Entry point
if __name__== '__main__':
    print("Takeshi STATE MACHINE...")
    init("takeshi_smach")
    sm = smach.StateMachine(outcomes = ['END'])     #State machine, final state "END"
    sm.userdata.sm_counter = 0
    sm.userdata.clear = False

    with sm:
        #State machine for Restaurant
        smach.StateMachine.add("INITIAL",           Initial(),          transitions = {'failed':'INITIAL',          'succ':'MAPPING',           'tries':'END'})
        smach.StateMachine.add("MAPPING",           Mapping(),          transitions = {'failed':'INITIAL',          'succ':'SCAN_RESTAURANT',   'tries':'END'})
        smach.StateMachine.add("SCAN_RESTAURANT",   Scan_restaurant(),  transitions = {'failed':'SCAN_RESTAURANT',  'succ':'GOTO_TABLE',        'tries':'INITIAL'})
        smach.StateMachine.add("GOTO_TABLE",        Goto_table(),       transitions = {'failed':'GOTO_TABLE',       'succ':'HRI_TAKE_ORDER',    'tries':'GOTO_TABLE', 'end':'INITIAL'})
        smach.StateMachine.add("HRI_TAKE_ORDER",    Hri_take_order(),   transitions = {'failed':'HRI_TAKE_ORDER',   'succ':'HRI_CONFIRM_ORDER', 'tries':'INITIAL'})
        smach.StateMachine.add("HRI_CONFIRM_ORDER", Hri_confirm_order(),transitions = {'failed':'HRI_TAKE_ORDER',   'succ':'GOTO_BAR',          'tries':'INITIAL'})
        smach.StateMachine.add("GOTO_BAR",          Goto_bar(),         transitions = {'failed':'GOTO_BAR',         'succ':'SCAN_BAR',          'tries':'END'})
        smach.StateMachine.add("SCAN_BAR",          Scan_bar(),         transitions = {'failed':'GOTO_BAR',         'succ':'PRE_GRASP_BAR',     'tries':'GOTO_BAR'})
        smach.StateMachine.add("PRE_GRASP_BAR",     Pre_grasp_bar(),    transitions = {'failed':'SCAN_BAR',         'succ':'GRASP_BAR',         'tries':'GOTO_BAR'})
        smach.StateMachine.add("GRASP_BAR",         Grasp_bar(),        transitions = {'failed':'GOTO_BAR',         'succ':'GOTO_DEL_GOAL',     'tries':'SCAN_BAR'})
        smach.StateMachine.add("GOTO_DEL_GOAL",     Goto_del_goal() ,   transitions = {'failed':'GOTO_DEL_GOAL',    'succ':'DELIVERY',          'tries':'GOTO_DEL_GOAL'})
        smach.StateMachine.add("DELIVERY",          Delivery() ,        transitions = {'failed':'DELIVERY',         'succ':'END',               'tries':'END'})

    outcome = sm.execute()

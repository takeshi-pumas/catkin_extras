#!/usr/bin/env python3
from smach_utils2 import *
from smach_ros import SimpleActionState
from action_server.msg import GraspAction
from std_msgs.msg import Float32MultiArray
import tf.transformations
#####
# Initialize target_object here
## REgions and known locs relative to TMR2024 world
class Initial(smach.State):
    def __init__(self):
        smach.State.__init__(self,output_keys=['target_object','placing_area','placing_area_quat'] , outcomes=['succ', 'failed', 'tries'])
        self.tries = 0
    def execute(self, userdata):        
        rospy.loginfo('STATE : INITIAL')
        print('Robot neutral pose')
        self.tries += 1
        print(f'Try {self.tries} of 5 attempts')
        if self.tries == 3:
            return 'tries'        
             
        global arm ,  hand_rgb     ,regions_df    
        #userdata.target_object='spoon'
        userdata.target_object='cereal_box'   #Strategy. pickup bowl first
        #userdata.target_object='milk'
        #userdata.target_object='bowl'
        hand_rgb = HAND_RGB()        
        rospack = rospkg.RosPack()        
        file_path = rospack.get_path('config_files')+'/regions'         
        
        
        #######################################
        #x,y,z= 5.8 , 1.3, 0.47   #SIM TMR  table plane
        #quat=[0.0,0.0,0.0,1.0]
        #o=read_yaml('/regions/regions_sim.yaml')#SIM (TMR WORLD)
        ##########################################
                
        #####################################
        x,y,z= -0.5 , -2.9, 0.8   #REAL LAB
        quat=[0.0,0.0,0.707,-0.707]
        o=read_yaml('/regions/regions.yaml')#REAL
        ###############################################


        regions_df=pd.DataFrame.from_dict(o)

        userdata.placing_area=[x,y,z]
        userdata.placing_area_quat=quat
        tf_man.pub_static_tf(pos=[x,y,z], rot=quat,point_name='placing_area') ### Ideal  coordinates to place Bowl
        
        arm = moveit_commander.MoveGroupCommander('arm')
        head.set_named_target('neutral')
        rospy.sleep(0.8)
        arm.set_named_target('go')
        arm.go()
        rospy.sleep(0.3)
        return 'succ'

#########################################################################################################
class Wait_push_hand(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):
        #############################################################################################
        #return 'succ'## REMOVE  THIS IS ONLY FOR GAZEBO TESTING (no push hand simulated just skip)
        #############################################################################################  
        rospy.loginfo('STATE : Wait for Wait_push_hand')
        print('Waiting for hand to be pushed')
        self.tries += 1
        print(f'Try {self.tries} of 4 attempts')
        if self.tries == 4:
            return 'tries'
        talk('Gently... push my hand to begin')           
        succ = wait_for_push_hand(100) # NOT GAZEBABLE
        if succ:
            return 'succ'
        else:
            return 'failed'
###########################################################################################################
class Goto_pickup(smach.State):  
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed'])
        self.tries=0

    def execute(self, userdata):
        
        rospy.loginfo('STATE : Navigate to known location')
        self.tries+=1
            
        rospy.sleep(1.5)
        res = omni_base.move_base(known_location='pickup', time_out=40)
        res = omni_base.move_base(known_location='pickup', time_out=40)
         

        if self.tries <= 1: 
            talk('Navigating to, pickup')
        if res:
            self.tries=0
            return 'succ'
        else:
            talk('Navigation Failed, retrying')
            return 'failed'


class Scan_table(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,  input_keys =['target_object'],outcomes=['succ', 'failed', 'tries'])
        self.tries = 0       
    def execute(self, userdata):
        rospy.loginfo('State : Scanning_table')
        print (f'target object is {userdata.target_object}')
        self.tries += 1
     
        if self.tries==1 and userdata.target_object != 'spoon':
            head.set_joint_values([ 0.0, -0.5])
            talk(f'Scanning Table , looking for {userdata.target_object}')
            print(f'Scanning Table , looking for {userdata.target_object}')
        if self.tries==2 and userdata.target_object != 'spoon':
            head.set_joint_values([ -0.13, -0.5])
            talk('Scanning Table')
            print(f'Scanning Table , looking for {userdata.target_object}')

        if self.tries==3 and userdata.target_object != 'spoon':
            head.set_joint_values([ 0.13, -0.5])

            print(f'Scanning Table , looking for {userdata.target_object}')
            talk('Scanning Table')
            self.tries=0
        if  userdata.target_object == 'spoon':
            #av=arm.get_current_joint_values() 
            #av[0]=0.05
            #arm.set_joint_value_target(av)
            #arm.go()
            rospy.sleep(1.0)
            head.set_joint_values([0.0, -0.5])
        
        print ('scanNING TABLE')

        
        rospy.sleep(3.0)                        
        img_msg  = bridge.cv2_to_imgmsg( cv2.cvtColor(rgbd.get_image(), cv2.COLOR_RGB2BGR))### Using Yolo to find [bowl,milk,cereal , spoon]
        req      = classify_client.request_class()
        req.in_.image_msgs.append(img_msg)
        res      = classify_client(req)
        objects=detect_object_yolo('all',res)   
        print (objects)
        common_misids=[]
        if userdata.target_object== 'bowl': common_misids.append('plate')
        elif userdata.target_object== 'cereal_box':
            common_misids.append('master_chef_can')
            common_misids.append('potted_meat_can')
            common_misids.append('pudding_box')
        elif userdata.target_object== 'milk':
            common_misids.append('bleach_cleanser')
            common_misids.append('sugar_box')
            #common_misids.append('master_chef_can')
        
        elif userdata.target_object== 'spoon':
            common_misids.append('fork')
            common_misids.append('knife')
            common_misids.append('large_marker')
        
        #####################
        area_bo_x=regions_df['pickup'][['x_min','x_max']].values
        area_bo_y=regions_df['pickup'][['y_min','y_max']].values
        pickup_plane_z=regions_df['pickup']['z']

        area_box=np.concatenate((area_bo_x,area_bo_y)).reshape((2,2)).T
        print (area_box)
        #####################
        def is_inside(x,y,z):return ((area_box[:,1].max() > y) and (area_box[:,1].min() < y)) and ((area_box[:,0].max() > x) and (area_box[0,0].min() < x)) #and (pickup_plane_z<z)  
        if len (objects)!=0 :
            _,quat_base= tf_man.getTF('base_link')  #  For grasping purposes object is orientated in front of base link.
            
            for i in range(len(res.poses)):
                    if res.names[i].data[4:]== userdata.target_object or  res.names[i].data[4:] in  common_misids:
                        #tf_man.getTF("head_rgbd_sensor_rgb_frame")
                        position = [res.poses[i].position.x ,res.poses[i].position.y,res.poses[i].position.z]
                        #print ('position,name',position,res.names[i].data[4:])
                        ##########################################################
                        object_point = PointStamped()
                        object_point.header.frame_id = "head_rgbd_sensor_rgb_frame"
                        object_point.point.x = position[0]
                        object_point.point.y = position[1]
                        object_point.point.z = position[2]
                        position_map = tfBuffer.transform(object_point, "map", timeout=rospy.Duration(1))
                        print ('position_map',position_map,'name' ,res.names[i].data[4:],is_inside(position_map.point.x,position_map.point.y,position_map.point.z))
                        if is_inside(position_map.point.x,position_map.point.y,position_map.point.z): 
                            tf_man.pub_static_tf(pos= [position_map.point.x,position_map.point.y,position_map.point.z], rot=quat_base, ref="map", point_name=userdata.target_object)#res.names[i].data[4:] )
                            self.tries=0
                            talk(f'{userdata.target_object} found , grasping')
                            return 'succ'
                        ###########################################################                
        print(f'Couldnt find {userdata.target_object}, will retry.')
        talk(f'Couldnt find {userdata.target_object}, will retry.')
        return 'failed'        
    
        #################################################################################################
class Place(smach.State):   
    def __init__(self):
        smach.State.__init__(self, input_keys =['target_object','placing_area','placing_area_quat'], output_keys=['target_pose','target_object','mode'],outcomes=['succ', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):
        rospy.loginfo('STATE : PLACE')
        current_target=userdata.target_object
        pos,i= False,0
        while( pos==False and i<10):
            i+=1
            if i>1:print ('why tf fails!?!??!', i)
            tf_man.pub_static_tf(pos=userdata.placing_area, rot=userdata.placing_area_quat,point_name='placing_area') ### 
            rospy.sleep(0.5)
            pos, quat = tf_man.getTF(target_frame = 'placing_area', ref_frame = 'odom')      
                
        string_msg= String()  #mode mesge new instance
        print (f'placing {current_target}')

        #current_target='bowl'
        common_misids=[]
        #################################################
        if current_target=='bowl':
            rospy.loginfo('STATE : PLACE BOWL')            
            print ('STATE : PLACE BOWL')                       
            offset_point=[0.04,-0.05,-0.031]          # Offset relative to object tf#                       
            string_msg.data='frontal'       
        elif current_target== 'spoon':
            string_msg.data='pour'               # Offset relative to object tf            


        ######################################################################
        else:                           #if current_target== 'cereal_box' or current_target== 'milk'  :
            string_msg.data='pour'               # Offset relative to object tf            
            offset_point=[-0.025,0.066,0.20]           # Offset relative to object tf#
            # LOOK FOR OBJECT current target
            if current_target== 'cereal_box':
                rospy.loginfo('STATE : POUR CEREAL')            
                print ('STATE : POUR CEREAL')
                talk( 'pouring cereal')     
            elif current_target== 'milk':
                rospy.loginfo('STATE : POUR MILK')            
                print ('STATE : POUR MILK')
                talk( 'pouring cereal')    
            head.set_named_target('neutral')
            rospy.sleep(0.5)       
            common_misids.append('plate')           #grasp_type 
            talk ('looking for bowl')
            rospy.sleep(3.0)  
                                 
            img_msg  = bridge.cv2_to_imgmsg( cv2.cvtColor(rgbd.get_image(), cv2.COLOR_RGB2BGR))### Using Yolo to find [bowl,milk,cereal , spoon]
            req      = classify_client.request_class()
            req.in_.image_msgs.append(img_msg)
            res      = classify_client(req)
            objects=detect_object_yolo('all',res)   
            print (objects)            
            if len (objects)!=0 :
                _,quat_base= tf_man.getTF('base_link')  
                for i in range(len(res.poses)):
                        if res.names[i].data[4:]== 'bowl' or  res.names[i].data[4:] in  common_misids:
                            #tf_man.getTF("head_rgbd_sensor_rgb_frame")
                            position = [res.poses[i].position.x ,res.poses[i].position.y,res.poses[i].position.z]
                            #print ('position,name',position,res.names[i].data[4:])
                            ##########################################################
                            object_point = PointStamped()
                            object_point.header.frame_id = "head_rgbd_sensor_rgb_frame"
                            object_point.point.x = position[0]
                            object_point.point.y = position[1]
                            object_point.point.z = position[2]
                            position_map = tfBuffer.transform(object_point, "map", timeout=rospy.Duration(1))
                            #print ('position_map',position_map)
                            tf_man.pub_static_tf(pos= [position_map.point.x,position_map.point.y,position_map.point.z], rot=quat_base, ref="map", point_name="placing_area")  #res.names[i].data[4:] )
                            rospy.sleep(0.5)
                            pos, quat = tf_man.getTF(target_frame = 'placing_area', ref_frame = 'odom')                  
                            talk('bowl found. pouring')   
                            line_up_TF('placing_area')

        #####################################################################
        userdata.mode=string_msg 
            ####################
            
        print (f'target_object(placng){current_target} , mode {string_msg.data}')
        translated_point = np.array(tf.transformations.quaternion_multiply(quat, offset_point + [0]))[:3] + np.array(pos)
        pose_goal=np.concatenate((translated_point,quat))
        print (f'transalted_point-<{pose_goal}')    
        target_pose = Float32MultiArray()
        target_pose.data = pose_goal#pos
        tf_man.pub_static_tf(pos= translated_point, rot=tf.transformations.quaternion_multiply(quat, [0,0,1,0]), ref="odom", point_name='goal_for_place' )  
        userdata.target_pose = target_pose
        print (f'transalted_point-<{target_pose.data}')    
        ###################
        head.set_named_target('neutral')
        rospy.sleep(0.5)       
        clear_octo_client()     
        self.tries+=1  


        
        return 'succ'
        
#########################################################################################################



class Pickup(smach.State):   
    def __init__(self):
        smach.State.__init__(self, output_keys=['target_pose','mode'], input_keys =['target_object'] ,outcomes=['succ', 'failed', 'tries'])
        self.tries = 0
    def execute(self, userdata):
        target_object= userdata.target_object
        line_up_TF(target_object)
        #print ( 'linning up')
        if target_object=='bowl' :           
            
            string_msg= String()
            string_msg.data='above'                
            userdata.mode=string_msg 
            rospy.loginfo('STATE : PICKUP BOWL')                                            
            pos, quat = tf_man.getTF(target_frame = target_object, ref_frame = 'odom')
            target_pose = Float32MultiArray()
            print (f'target_object {target_object}')        
            print ( 'Applying Bowl Offset') 
            offset_point=[0.03,-0.05,+0.08]   # Offset relative to object tf
        ######################################################################
        if target_object=='cereal_box'or target_object=='milk':
            
            string_msg= String()
            string_msg.data='pour'
            userdata.mode=string_msg 
            rospy.loginfo('STATE : PICKUP CEREAL')            
            print ('STATE : PICKUP CEREAL')            
            pos, quat = tf_man.getTF(target_frame = target_object, ref_frame = 'odom')
            print (f'target_object {target_object}, mode {string_msg.data}')
            ####################                        
            line_up_TF(target_object)####################
            print ( 'linning up') 
            userdata.mode=string_msg 
            pos, quat = tf_man.getTF(target_frame = target_object, ref_frame = 'map')
            print (f'target_object {target_object}, mode {string_msg.data}')
            ####################
            offset_point=[-0.09,0.0,0.01]          # Offset relative to object tf
            ####################                        
        if target_object=='spoon':
            pos, quat = tf_man.getTF(target_frame = target_object, ref_frame = 'map')
            string_msg= String()
            string_msg.data='above'                
            userdata.mode=string_msg 
            offset_point=[-0.1,0.0,+0.09]  
            rospy.loginfo('STATE : PICKUP SPOON')            
            print ('STATE : PICKUP SPOON')
        
        
        #####################APPLY OFFSET
        object_point = PointStamped()
        object_point.header.frame_id = target_object#"base_link"
        object_point.point.x = offset_point[0]
        object_point.point.y = offset_point[1]
        object_point.point.z = offset_point[2]
        transformed_object_point = tfBuffer.transform(object_point, "map", timeout=rospy.Duration(1))
        tf_man.pub_static_tf(pos=[transformed_object_point.point.x,transformed_object_point.point.y,transformed_object_point.point.z],rot=quat,point_name='goal_for_grasp')
        ##################################################################3
        rospy.sleep(0.5)
        pos, quat = tf_man.getTF(target_frame = 'goal_for_grasp', ref_frame = 'odom')
        pose_goal=np.concatenate((pos,quat))
        print (f'transalted_point-<{pose_goal}')    
        target_pose = Float32MultiArray()
        target_pose.data = pose_goal#pos
        #tf_man.pub_static_tf(pos= translated_point, rot=tf.transformations.quaternion_multiply(quat, [0,0,1,0]), ref="odom", point_name='goal_for_grasp' )   
        rospy.sleep(0.5)
        userdata.target_pose = target_pose
        print (f'transalted_point-<{target_pose.data}')    
        ###################
        head.set_named_target('neutral')
        rospy.sleep(0.5)       
        clear_octo_client()     
        self.tries+=1  
        return 'succ'
        
        
class Goto_breakfast(smach.State):  
    def __init__(self):
        smach.State.__init__(self, input_keys =['target_object'], outcomes=['succ', 'failed', 'tries', 'bowl_place'])
        self.tries = 0
    def execute(self, userdata):
        rospy.loginfo('STATE : Navigate to known location')
        
        print(f'Try {self.tries} of 3 attempts')
        self.tries += 1
        if self.tries == 3:
            return 'tries'
        #omni_base.tiny_move( velX=-0.2,std_time=4.2) 
        if self.tries == 1: talk('Navigating to, breakfast')
        res = omni_base.move_base(known_location='breakfast', time_out=150)
        print (res)
        res = omni_base.move_base(known_location='breakfast', time_out=50)
        print (res)
        #res = omni_base.move_base(known_location='breakfast', time_out=50)
        #print (res)
        if res:
            self.tries=0
            if userdata.target_object=='bowl':return 'bowl_place'
            return 'succ'
        else:
            talk('Navigation Failed, retrying')
            return 'failed'
#########################################################################################################
class Goto_place_breakfast(smach.State):  
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):

        rospy.loginfo('STATE : Navigate to known location')

        print(f'Try {self.tries} of 3 attempts')
        self.tries += 1
        if self.tries == 3:
            return 'tries'
        arm.set_named_target('neutral')    
        arm.set_named_target('go')    
        arm.go()
        if self.tries == 1: talk('Navigating to, breakfast table')
        res = omni_base.move_base(known_location='breakfast', time_out=10)
        print(res)

        if res:
            return 'succ'
        else:
            talk('Navigation Failed, retrying')
            return 'failed'

class Place_post_pour(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'continue'], input_keys =['target_object','placing_area','placing_area_quat','mode'], output_keys=['target_pose','target_object','mode'] )
        

    def execute(self, userdata):
        print (userdata.mode)
        
        if userdata.mode.data   != 'pour':
            gripper.steady()
            brazo.set_named_target('go')
            rospy.sleep(2.0) # BRAZO FACE PALM AVOIDANCE
            print ('## CONTINUE TO NEXT OBJECT')
            return 'continue'
        rospy.loginfo('STATE : PLACE POST POUR')
        print('STATE : PLACE POST POUR')
        
        target_object=userdata.target_object
        #if userdata.target_object=='cereal_box':
        #    pose_target[0]+=-0.35
        #    userdata.target_object='milk'
        #if userdata.target_object=='milk':
        #    pose_target[0]+=-0.45
        #    userdata.target_object='spoon'

        _,quat=tf_man.getTF('placing_area')
        line_up_TF('placing_area')                   
        string_msg= String()  #mode mesge new instance
        rospy.loginfo('STATE : PLACE AFTER POUR')            
        print ('STATE : PLACE AFTER POUR')                       
        if userdata.target_object=='cereal_box':offset_point=[0.0,-0.45,-0.031]          # Offset relative to object tf#
        else:offset_point=[0.0,-0.25,-0.04 ]
        omni_base.tiny_move(velY =-0.2, std_time=4.2)
        string_msg.data='frontal'
        userdata.mode=string_msg             
        ###################
        #####################APPLY OFFSET
        object_point = PointStamped()
        object_point.header.frame_id = "placing_area"    ##userdata.target_object#"base_link"
        object_point.point.x = offset_point[0]
        object_point.point.y = offset_point[1]
        object_point.point.z = offset_point[2]
        transformed_object_point = tfBuffer.transform(object_point, "map", timeout=rospy.Duration(1))
        tf_man.pub_static_tf(pos=[transformed_object_point.point.x,transformed_object_point.point.y,transformed_object_point.point.z],rot=quat,point_name='goal_for_place')
        ##################################################################3
        rospy.sleep(0.5)
        pos, quat = tf_man.getTF(target_frame = 'goal_for_place', ref_frame = 'odom')
        pose_goal=np.concatenate((pos,quat))
        print (f'transalted_point-<{pose_goal}')    
        target_pose = Float32MultiArray()
        target_pose.data = pose_goal#pos
        #tf_man.pub_static_tf(pos= translated_point, rot=tf.transformations.quaternion_multiply(quat, [0,0,1,0]), ref="odom", point_name='goal_for_grasp' )   
        rospy.sleep(0.5)
        userdata.target_pose = target_pose
        print (f'transalted_point-<{target_pose.data}')    
        ###################
        head.set_named_target('neutral')
        rospy.sleep(0.5)       
        clear_octo_client()     
        if target_object=='cereal_box':
            userdata.target_object='milk'
            talk (f'Now going for {userdata.target_object } ')
        if target_object=='milk':
            userdata.target_object='spoon'
            talk (f'Now going for {userdata.target_object } ')
        return 'succ'
        

        

#########################################################################################################
class Wait_door_opened(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed'])
        self.tries = 0

    def execute(self, userdata):

        rospy.loginfo('STATE : Wait for door to be opened')
        print('Waiting for door to be opened')

        self.tries += 1
        print(f'Try {self.tries} of 4 attempts')

        # if self.tries == 100:
        #     return 'tries'
        if self.tries == 1 : talk('I am ready for Serve Breakfast task.')
        rospy.sleep(0.8)
        talk('I am waiting for the door to be opened')
        isDoor = line_detector.line_found()
        #succ = wait_for_push_hand(100)
        rospy.sleep(1.0)
        if not isDoor:
            self.tries = 0
            return 'succ'
        else:
            return 'failed'

#########################################################################################################
class Pour(smach.State):
    def __init__(self):
        smach.State.__init__(self,input_keys=['target_object','placing_area'], outcomes=['succ', 'failed'])
        self.tries = 0

    def execute(self, userdata):

        rospy.loginfo('STATE : pouring ')
        print('Pouring')

        self.tries += 1
        print(f'Try {self.tries} of 4 attempts')
        av=arm.get_current_joint_values()
        av[0]+=-0.25
        av[-2]+=1.7
        succ=arm.go(av)
        rospy.sleep(2.0)
        av=arm.get_current_joint_values()
        av[0]+=0.25
        av[-2]=-1.9
        succ=arm.go(av)
        
        if userdata.target_object=='cereal_box':target_placing= 'placing_cereal'
        if userdata.target_object=='milk':target_placing= 'placing_milk'
        hand_grasp_D(target_placing)  

        if succ:
            return 'succ'
        else:
            return 'failed'

#########################################################################################################
class Place_breakfast(smach.State):  
    def __init__(self):
        smach.State.__init__(self,output_keys=['target_object','placing_area','placing_area_quat'],input_keys=['target_object','placing_area','placing_area_quat'], outcomes=['succ', 'failed', 'tries','pour'])
        self.tries = 0
    def execute(self, userdata):
        rospy.loginfo('STATE : Placing ')
        armlift=userdata.placing_area[2]-0.12  # Takeshi shoulder to placing table heigh offset for placing bowl
        place_pose= [armlift,-1.0, 0.04852065465630595, -1.8, 0.07179310822381613,0.0]  # ARM PREDEFINED PALCING POSE 
        print ('place pose',place_pose)
        if userdata.target_object=='cereal_box':place_pose[3]=-0.532
        arm.set_joint_value_target(place_pose)
        arm.go()

        print ('av',arm.get_current_joint_values())
        pos, quat = tf_man.getTF(target_frame = 'placing_area', ref_frame = 'odom')      
        i=0

        while( pos==False and i<10):
            i+=1
            if i>1:print ('why tf fails!?!??!', i)
            tf_man.pub_static_tf(pos=userdata.placing_area, rot=userdata.placing_area_quat,point_name='placing_area') ### Ideal  coordinates to place Bowl
            rospy.sleep(0.5)
            pos, quat = tf_man.getTF(target_frame = 'placing_area', ref_frame = 'odom')   

                
        base_grasp_D(tf_name='placing_area',d_x=0.4, d_y=0.0,timeout=30)
        if userdata.target_object=='bowl':
            clear_octo_client()
            av=arm.get_current_joint_values()
            print (av)

            av[1]=-1.6       
            arm.set_joint_value_target(av)
 
            succ=arm.go()
            print (av)

            if not succ:                
                av[1]=-1.57        ## ARM FELX JOINT TO PLACE
                sec_chance=arm.go(av)
                if not sec_chance:
                        
                        brazo.set_joint_values([0.6808629824410867,-1.57, 0.04852065465630595, -1.8, 0.07179310822381613])
                        rospy.sleep(1)
            

        bowl_pose,_= tf_man.getTF('hand_palm_link')
        _,quat_base= tf_man.getTF('base_link')  
        
        bowl_pose[2]= userdata.placing_area[2]  ## KEEP SAME PREDIFINDE PLANE PLACING HEIGHT
        userdata.placing_area_quat= quat_base
        userdata.placing_area= bowl_pose
        print (bowl_pose)      
        gripper.open()
        rospy.sleep(2.0)
        escape_pose= [0.6808629824410867,-1.2, 0.04852065465630595, -1.3035706302849657, 0.07179310822381613,0.0]
        arm.set_joint_value_target(escape_pose)
        arm.go()        
        base_grasp_D(tf_name='placing_area',d_x=0.8, d_y=0.0,timeout=30)
        #omni_base.tiny_move( velX=-0.3,std_time=7.2) 
 
        rospy.sleep(2.0)
        gripper.steady()
        arm.set_named_target('go')
        succ=arm.go()        
        if succ:
            userdata.target_object='cereal_box'
            return'succ'
        else:
            clear_octo_client()
            omni_base.tiny_move( velX=-0.3,std_time=3.0) 
            arm.set_named_target('go')
            succ=arm.go()
            if succ:
                userdata.target_object='cereal_box'
                return'succ'
        return 'failed'



        
        
        
#########################################################################################################
class Check_grasp(smach.State):   
    def __init__(self):
        smach.State.__init__(self,input_keys=['target_object','target_pose','mode'], output_keys=['target_pose','mode'],outcomes=['succ', 'failed', 'tries'])
        self.tries=0

    def execute(self, userdata):
        self.tries+=1
        if self.tries>=4:
            self.tries=0
            return 'tries'
        rospy.loginfo('STATE : Check Grasp')
        print(userdata.mode, userdata.target_pose)
        

        arm.set_named_target('go')

        arm.go()
        head.to_tf(userdata.target_object)
        rospy.sleep(1.0)
        img_msg  = bridge.cv2_to_imgmsg( cv2.cvtColor(rgbd.get_image(), cv2.COLOR_RGB2BGR))### GAZEBO BGR!?!??!
        req      = classify_client.request_class()
        req.in_.image_msgs.append(img_msg)
        res      = classify_client(req)
        objects=detect_object_yolo('all',res)   

        def check_if_grasped(pose_target,test_pt,tolerance=0.05):return np.linalg.norm(pose_target-test_pt)<tolerance
        ##############################
        pose_target,_=tf_man.getTF(userdata.target_object)
        #########################

        if len (objects)!=0 :
            for i in range(len(res.poses)):
                
                position = [res.poses[i].position.x ,res.poses[i].position.y,res.poses[i].position.z]
                
                object_point = PointStamped()
                object_point.header.frame_id = "head_rgbd_sensor_rgb_frame"
                object_point.point.x = position[0]
                object_point.point.y = position[1]
                object_point.point.z = position[2]
                position_map = tfBuffer.transform(object_point, "map", timeout=rospy.Duration(1))
                tf_man.pub_static_tf(pos= [position_map.point.x,position_map.point.y,position_map.point.z], rot=[0,0,0,1], ref="map", point_name=res.names[i].data[4:] )             
                test_pt=np.asarray((position_map.point.x,position_map.point.y,position_map.point.z))
                print (np.linalg.norm(pose_target-test_pt))
                if check_if_grasped(pose_target,test_pt):
                    print (f'Centroid found in area {test_pt}, obj_name: {res.names[i].data[4:]}')
                    print ('Grasping May have failed')
                    print (f'recalling grasp action on coordinates{test_pt} wrt map, converting to odom and action goal slot  ')
                    pos, _ = tf_man.getTF(target_frame = userdata.target_object, ref_frame = 'odom')
                    return 'failed'
        
        self.tries=0
        
        return'succ'    
                
                
                
                
                


        

# --------------------------------------------------
def init(node_name):
       print('smach ready')
   # --------------------------------------------------
# Entry point
if __name__ == '__main__':
    print("Takeshi STATE MACHINE...")
    init("takeshi_smach")
    # State machine, final state "END"
    sm = smach.StateMachine(outcomes=['END'])
    # sm.userdata.clear = False
    sis = smach_ros.IntrospectionServer('SMACH_VIEW_SERVER', sm, '/SM_STORING')
    sis.start()
    with sm:
        # State machine STICKLER
        smach.StateMachine.add("INITIAL",           Initial(),              transitions={'failed': 'INITIAL',           
                                                                                         'succ': 'GOTO_PICKUP',   
                                                                                         'tries': 'END'})
        smach.StateMachine.add("WAIT_PUSH_HAND",    Wait_push_hand(),       transitions={'failed': 'WAIT_PUSH_HAND',    
                                                                                         'succ': 'GOTO_PICKUP',       
                                                                                         'tries': 'END'})
        smach.StateMachine.add("WAIT_DOOR",    Wait_door_opened(),       transitions={'failed': 'WAIT_DOOR',    
                                                                                         'succ': 'GOTO_PICKUP'})
        ####################################################################################################################
        smach.StateMachine.add("GOTO_PICKUP",    Goto_pickup(),       transitions={'failed': 'GOTO_PICKUP',    
                                                                                         'succ': 'SCAN_TABLE'})
        smach.StateMachine.add("SCAN_TABLE",    Scan_table(),       transitions={'failed': 'SCAN_TABLE',    
                                                                                         'succ': 'PICKUP',       
                                                                                         'tries': 'GOTO_PICKUP'})        
        smach.StateMachine.add("GOTO_BREAKFAST",    Goto_breakfast(),       transitions={'failed': 'GOTO_BREAKFAST',    
                                                                                         'succ': 'PLACE',#_BREAKFAST',       
                                                                                         'tries': 'GOTO_BREAKFAST',
                                                                                         'bowl_place':'PLACE_BREAKFAST'})
        
        smach.StateMachine.add("PLACE_BREAKFAST",    Place_breakfast(),       transitions={'failed': 'PLACE_BREAKFAST',    
                                                                                         'succ': 'GOTO_PICKUP',       
                                                                                         'tries': 'GOTO_BREAKFAST',
                                                                                         'pour':'POUR'})
        smach.StateMachine.add("POUR",    Pour(),       transitions={'failed': 'POUR',  'succ': 'GOTO_PICKUP'})
        
        smach.StateMachine.add("PICKUP",    Pickup(),       transitions={'failed': 'PICKUP',    
                                                                                         'succ': 'GRASP_GOAL',       
                                                                                         'tries': 'GOTO_PICKUP'})
        smach.StateMachine.add("CHECK_GRASP",    Check_grasp(),       transitions={'failed': 'GRASP_GOAL',    
                                                                                         'succ': 'GOTO_BREAKFAST',
                                                                                         'tries': 'GOTO_PICKUP'})
        smach.StateMachine.add("GRASP_GOAL", SimpleActionState('grasp_server', GraspAction, goal_slots={'target_pose': 'target_pose', 'mode': 'mode'}),                      
                        transitions={'preempted': 'END', 'succeeded': 'CHECK_GRASP', 'aborted': 'CHECK_GRASP'})
        
        smach.StateMachine.add("PLACE",    Place(),       transitions={'failed': 'PLACE',    
                                                                                         'succ': 'PLACE_GOAL',
                                                                                         'tries': 'GOTO_BREAKFAST'})        
        
        smach.StateMachine.add("PLACE_GOAL", SimpleActionState('place_server', GraspAction, goal_slots={'target_pose': 'target_pose', 'mode': 'mode'}),              
                        transitions={'preempted': 'END', 'succeeded': 'PLACE_POST_POUR', 'aborted': 'GOTO_PICKUP'})
        
        
        smach.StateMachine.add("PLACE_POST_POUR",    Place_post_pour(),       transitions={'continue': 'GOTO_PICKUP',    
                                                                                         'succ': 'PLACE_GOAL'})
                
        ###################################################################################################################################################################
        


    outcome = sm.execute()

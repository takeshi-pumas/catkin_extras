#!/usr/bin/env python3
from smach_utils2 import *
from smach_ros import SimpleActionState
from action_server.msg import GraspAction
from std_msgs.msg import Float32MultiArray
#####
# Initialize target_object here
## REgions and known locs relative to TMR2024 world
class Initial(smach.State):
    def __init__(self):
        smach.State.__init__(self,output_keys=['target_object'] , outcomes=['succ', 'failed', 'tries'])
        self.tries = 0
    def execute(self, userdata):        
        rospy.loginfo('STATE : INITIAL')
        print('Robot neutral pose')
        self.tries += 1
        print(f'Try {self.tries} of 5 attempts')
        if self.tries == 3:
            return 'tries'        
             
        global arm ,  hand_rgb 

        #Strategy. pickup bowl first
        #userdata.target_object='potted_meat_can'
        userdata.target_object='bowl'
        hand_rgb = HAND_RGB()
        x,y,z= -0.7 , .46, 0.8
        tf_man.pub_static_tf(pos=[x,y,z],point_name='placing_area') ### Ideal  coordinates to place Bowl
        y= 0.66
        tf_man.pub_static_tf(pos=[x,y,z],point_name='placing_cereal') ### Ideal  coordinates to place Cereal
        y= 0.86                                      
        tf_man.pub_static_tf(pos=[x,y,z],point_name='placing_milk') ### Ideal  coordinates to place Cereal
        y= 0.26
        tf_man.pub_static_tf(pos=[x,y,z],point_name='placing_spoon') ### Ideal  coordinates to place Bowl
        
        
        
        #############################SIM
        #x,y,z= 5.8 , 1.3, 0.78
        #tf_man.pub_static_tf(pos=[x,y,z],point_name='placing_area') ### Ideal  coordinates to place Bowl
        #y= 1.1
        #tf_man.pub_static_tf(pos=[x,y,z],point_name='placing_cereal') ### Ideal  coordinates to place Cereal
        #y= 0.9                                      
        #tf_man.pub_static_tf(pos=[x,y,z],point_name='placing_milk') ### Ideal  coordinates to place Cereal
        #y= 1.5
        #tf_man.pub_static_tf(pos=[x,y,z],point_name='placing_spoon') ### Ideal  coordinates to place Bowl
        ############################
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
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries','pickup'])
        self.tries = 0
        self.First=True

    def execute(self, userdata):
        rospy.loginfo('STATE : Navigate to known location')
        print(f'Try {self.tries} of 3 attempts')
            
        self.tries += 1
        if self.tries == 3:
            self.tries = 0
            return 'tries'        
        res = omni_base.move_base(known_location='pickup', time_out=40)
        print(res)
        res = omni_base.move_base(known_location='pickup', time_out=40)
        print(res)
        res = omni_base.move_base(known_location='pickup', time_out=40)
        print(res)
##################################################################First TIme Only go        
        if self.tries == 1: 
            talk('Navigating to, pickup')
            if res:
                return 'succ'
            else:
                talk('Navigation Failed, retrying')
                return 'failed'
#########################################################################################################
        if self.tries > 1: 
            talk('Planing pick  up ')
            if res:
                return 'pickup'
            else:
                talk('Navigation Failed, retrying')
                return 'failed'
#########################################################################################################

class Scan_table(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,  input_keys =['target_object'],outcomes=['succ', 'failed', 'tries'])
        self.tries = 0       
    def execute(self, userdata):
        rospy.loginfo('State : Scanning_table')
        print (f'target object is {userdata.target_object}')
        self.tries += 1
        if self.tries >= 3:
            self.tries = 0            
            print ( "I could not find more objects ")
            return 'tries'
        if self.tries==1:
            head.set_joint_values([ 0.0, -0.5])
            talk('Scanning Table')
        if self.tries==2:
            head.set_joint_values([ -0.3, -0.3])
            talk('Scanning Table')
        
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
        elif userdata.target_object== 'potted_meat_can': common_misids.append('pudding_box')
        if len (objects)!=0 :

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
                        #print ('position_map',position_map)
                        tf_man.pub_static_tf(pos= [position_map.point.x,position_map.point.y,position_map.point.z], rot=[0,0,0,1], ref="map", point_name=userdata.target_object)#res.names[i].data[4:] )
                        self.tries=0
                        return 'succ'
                        ###########################################################                
        print(f'Couldnt find {userdata.target_object}, will retry.')
        talk(f'Couldnt find {userdata.target_object}, will retry.')
        return 'failed'        
    
        
#########################################################################################################

class Pickup(smach.State):   
    def __init__(self):
        smach.State.__init__(self, output_keys=['target_pose'], input_keys =['target_object'] ,outcomes=['succ', 'failed', 'tries'])
        self.tries = 0
    def execute(self, userdata):
        target_object= userdata.target_object
        if target_object=='bowl':
            rospy.loginfo('STATE : PICKUP BOWL')
                                  
            

            pos, _ = tf_man.getTF(target_frame = target_object, ref_frame = 'odom')
            target_pose = Float32MultiArray()
            print (f'target_object {target_object}')
            if target_object=='bowl' or target_object=='plate':
                print ( 'Applying Bowl Offset') #TODO DIctionary
                pos[0] +=-0.05
                pos[1] += -0.02
                pos[2] += 0.03
                


                #pos[0] +=-0.04 ##  REAL WORLD
                #pos[1] += 0.04 ## BOWL OFFSET
                #pos[2] += 0.03

        if target_object=='potted_meat_can':
            
            rospy.loginfo('STATE : PICKUP CEREAL')            
            print ('STATE : PICKUP CEREAL')
            
            pos, _ = tf_man.getTF(target_frame = target_object, ref_frame = 'odom')
            if pos == False:
                self.tries+=1
                print (f'no {target_object} tf found, trying common misids')
                return 'failed'
                ## CEREAL OFFSET
                pos[2] += 0.1

        target_pose = Float32MultiArray()
        
        rob_pos,_=tf_man.getTF('base_link')
        print ("talk ( f'picking up {target_object}')")
        talk ( f'picking up {target_object}')
        ##################################################
        target_pose.data = pos
        userdata.target_pose = target_pose
        ###################
        head.set_named_target('neutral')
        rospy.sleep(0.5)       
        clear_octo_client()     
        self.tries+=1  
        return 'succ'
        
class Goto_breakfast(smach.State):  
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
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
       
        if res:
            self.tries=0
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
        if self.tries == 1 : talk('I am ready for Storing Groceries task.')
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
        smach.State.__init__(self,input_keys=['target_object'], outcomes=['succ', 'failed'])
        self.tries = 0

    def execute(self, userdata):

        rospy.loginfo('STATE : pouring ')
        print('Pouring')

        self.tries += 1
        print(f'Try {self.tries} of 4 attempts')
        
        av=arm.get_current_joint_values()
        #av[0]+=-0.25
        av[-2]=1.7
        succ=arm.go(av)
        if succ:        
            av=arm.get_current_joint_values()
            #av[0]+=0.25
            av[-2]+=-1.9
            arm.go(av)
        if userdata.target_object=='potted_meat_can':target_placing= 'placing_cereal'
        if userdata.target_object=='milk':target_placing= 'placing_milk'
        hand_grasp_D(target_placing)  

        if succ:
            return 'succ'
        else:
            return 'failed'

#########################################################################################################
class Place_breakfast(smach.State):  
    def __init__(self):
        smach.State.__init__(self,output_keys=['target_object'],input_keys=['target_object'], outcomes=['succ', 'failed', 'tries','pour'])
        self.tries = 0
    def execute(self, userdata):
        rospy.loginfo('STATE : Placing ')
        #place_pose= [0.390130913590598, -1.4407346556484901, 0.06971320811099346, -1.574294947826301, 0.0003442697253825955]  #SIM
        head.set_named_target('neutral')
        rospy.sleep(0.5)
        clear_octo_client()
        place_pose= [0.6808629824410867,-1.0, 0.04852065465630595, -1.8, 0.07179310822381613,0.0]
        if userdata.target_object=='potted_meat_can':place_pose[3]=-0.532
        arm.set_joint_value_target(place_pose)
        arm.go()
        
        base_grasp_D(tf_name='placing_area',d_x=0.4, d_y=0.0,timeout=30)
        if userdata.target_object=='bowl':
            clear_octo_client()
            av=arm.get_current_joint_values()
            av[1]=-1.59        
            succ=arm.go(av)
            if not succ:                
                av[1]=-1.57        
                sec_chance=arm.go(av)
                if not sec_chance:
                        
                        brazo.set_joint_values([0.6808629824410867,-1.57, 0.04852065465630595, -1.8, 0.07179310822381613])
                        rospy.sleep(1)



        if userdata.target_object=='potted_meat_can':
            pre_pour_pose=[0.68, -1.57, -0.04, -0.05, 0.07, 0.0]
            arm.set_joint_value_target(pre_pour_pose)
            arm.go()
            hand_grasp_D('placing_area')  

            return 'pour'

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
            userdata.target_object='potted_meat_can'
            return'succ'
        else:
            clear_octo_client()
            omni_base.tiny_move( velX=-0.3,std_time=3.0) 
            arm.set_named_target('go')
            succ=arm.go()
            if succ:
                userdata.target_object='potted_meat_can'
                return'succ'




        
        
        
#########################################################################################################
class Check_grasp(smach.State):   
    def __init__(self):
        smach.State.__init__(self,input_keys=['target_object'], output_keys=['target_pose'],outcomes=['succ', 'failed', 'tries'])
        self.tries=0

    def execute(self, userdata):
        self.tries+=1
        if self.tries>=4:
            self.tries=0
            return 'tries'
        rospy.loginfo('STATE : Check Grasp')
        

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
                    target_pose = Float32MultiArray()
                    #pos[2] += 0.03
                    target_pose.data = pos
                    userdata.target_pose = target_pose
                    ###################
                    head.set_named_target('neutral')
                    rospy.sleep(0.5)
                    clear_octo_client()
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
                                                                                         'succ': 'WAIT_PUSH_HAND',   
                                                                                         'tries': 'END'})
        smach.StateMachine.add("WAIT_PUSH_HAND",    Wait_push_hand(),       transitions={'failed': 'WAIT_PUSH_HAND', 
                                                                                         'succ': 'GOTO_PICKUP',       
                                                                                         'tries': 'END'})
        smach.StateMachine.add("WAIT_DOOR",    Wait_door_opened(),       transitions={'failed': 'WAIT_DOOR',    
                                                                                         'succ': 'GOTO_PICKUP'})
        
        smach.StateMachine.add("GOTO_PICKUP",    Goto_pickup(),       transitions={'failed': 'GOTO_PICKUP',    
                                                                                         'succ': 'SCAN_TABLE',       
                                                                                         'tries': 'GOTO_PICKUP',
                                                                                         'pickup':'PICKUP'})
        smach.StateMachine.add("SCAN_TABLE",    Scan_table(),       transitions={'failed': 'SCAN_TABLE',    
                                                                                         'succ': 'GOTO_PICKUP',       
                                                                                         'tries': 'GOTO_PICKUP'})        
        smach.StateMachine.add("GOTO_BREAKFAST",    Goto_breakfast(),       transitions={'failed': 'GOTO_BREAKFAST',    
                                                                                         'succ': 'PLACE_BREAKFAST',       
                                                                                         'tries': 'GOTO_BREAKFAST'})
        
        smach.StateMachine.add("PLACE_BREAKFAST",    Place_breakfast(),       transitions={'failed': 'PLACE_BREAKFAST',    
                                                                                         'succ': 'GOTO_PICKUP',       
                                                                                         'tries': 'GOTO_BREAKFAST',
                                                                                         'pour':'POUR'})
        

        smach.StateMachine.add("POUR",    Pour(),       transitions={'failed': 'POUR',    
                                                                                         'succ': 'END'})
        smach.StateMachine.add("PICKUP",    Pickup(),       transitions={'failed': 'PICKUP',    
                                                                                         'succ': 'GRASP_GOAL',       
                                                                                         'tries': 'GOTO_PICKUP'})
        smach.StateMachine.add("CHECK_GRASP",    Check_grasp(),       transitions={'failed': 'GRASP_GOAL',    
                                                                                         'succ': 'GOTO_BREAKFAST',
                                                                                         'tries': 'GOTO_PICKUP'})
        smach.StateMachine.add("GRASP_GOAL", SimpleActionState('grasp_server', GraspAction, goal_slots=['target_pose']),              
                        transitions={'preempted': 'END', 'succeeded': 'CHECK_GRASP', 'aborted': 'CHECK_GRASP'})
        
        ###################################################################################################################################################################
        


    outcome = sm.execute()

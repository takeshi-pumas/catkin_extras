#!/usr/bin/env python3
from lugg_utils import *
from smach_ros import SimpleActionState
from std_msgs.msg import String

#########################################################################################################
class Initial(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):

        
        rospy.loginfo('STATE : INITIAL')
        print('Robot neutral pose')
        self.tries += 1
        print(f'[INITIAL] Try {self.tries} of 5 attempts')
        if self.tries == 3:
            return 'tries'
        
        #READ YAML ROOMS XYS
        
        global arm #,  hand_rgb
        #hand_rgb = HAND_RGB()

        arm = moveit_commander.MoveGroupCommander('arm')
        head.set_named_target('neutral')
        rospy.sleep(0.8)
        arm.set_named_target('go')
        arm.go()
        rospy.sleep(0.3)

        #gripper.open()
        #rospy.sleep(0.3)

        #gripper.close()
        #rospy.sleep(0.3)

        
        return 'succ'

#########################################################################################################
class Wait_push_hand(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):

        rospy.loginfo('STATE : Wait for Wait_push_hand')
        print('Waiting for hand to be pushed')

        self.tries += 1
        print(f'Try {self.tries} of 4 attempts')
        if self.tries == 4:
            return 'tries'
        talk('Gently... push my hand to begin')
        
        
        succ = wait_for_push_hand(100) # NOT GAZEBABLE
        if succ:
            talk('Starting Carry my luggage task')
            return 'succ'
        else:
            return 'failed'

#########################################################################################################
class Goto_living_room(smach.State):  
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):

        rospy.loginfo('STATE : Navigate to known location')

        print(f'Try {self.tries} of 3 attempts')
        self.tries += 1
        if self.tries == 3:
            return 'tries'
        if self.tries == 1: #talk('Navigating to, living room')
            rospy.sleep(0.8)
        #res = omni_base.move_base(known_location='living_room', time_out=200)
        head.set_named_target('neutral')        
        rospy.sleep(2.0)
        res = True
        print(res)
        if res:
            return 'succ'
        else:
            talk('Navigation Failed, retrying')
            return 'failed'

#########################################################################################################
class Find_human(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=['succ', 'failed', 'tries'], output_keys=['second_pointing'])
        self.tries = 0
        self.point_img_pub = rospy.Publisher("/image_topic_name", String, queue_size=1)
        self.dist=2.5
    def execute(self, userdata):
        rospy.loginfo('State : Find_human')
        rospy.sleep(1.5)
        
        self.tries += 1
        if self.tries >= 5:
            self.tries = 0
            return 'tries'
        if self.tries==1:
            talk('Scanning the room for humans')
            head.set_joint_values([ 0.0, 0.0])# Looking ahead
        if self.tries==2:head.set_joint_values([ 0.5, 0.1])#looking left
        if self.tries==3:head.set_joint_values([-0.5, 0.1])#looking right        
        rospy.sleep(1.0)
        humanpose=detect_human_to_tf(dist = self.dist,remove_bkg=True)  #make sure service is running (pointing detector server now hosts this service)
        if humanpose== False:
            print ('no human ')
            return 'failed'
        
        talk('Please start pointing at the bag.')
        point_msg = String("pointingBAG.gif")
        self.point_img_pub.publish(point_msg)
        rospy.sleep(1.2)
        talk("In three")
        rospy.sleep(0.2)
        talk('Two')
        rospy.sleep(0.2)
        talk('One')
        rospy.sleep(0.2)
        self.point_img_pub.publish(String())
        rospy.sleep(0.1)

        req = Point_detectorRequest()
        req.dist = self.dist
        req.removeBKG = True        # CAMBIAR SI SE QUIERE QUITAR EL FONDO (CHISMOSOS) O NO
        res=pointing_detect_server(req)

        
        print("RES",res.x_r,res.x_l,res.y_r,res.y_l)

        hum_pos,_=tf_man.getTF('human')
        talk('Ok')
        rospy.sleep(0.8)
        threshold_bag_to_human=2.0  
        userdata.second_pointing=False
        ################################################################
        ########################################################
        if   res.x_r ==-1 :
            d_r=np.linalg.norm(hum_pos[:2]-np.asarray((res.x_r,res.y_r)))
            if d_r < threshold_bag_to_human:
                print("Only left hand")
                tf_man.pub_static_tf(pos=[res.x_l, res.y_l,0], rot =[0,0,0,1], point_name='pointing_')
                return 'succ'
        elif res.x_l ==-1:  
            d_l=np.linalg.norm(hum_pos[:2]-np.asarray((res.x_l,res.y_l)))
            if d_l < threshold_bag_to_human:
                print("Only right hand")
                tf_man.pub_static_tf(pos=[res.x_r, res.y_r,0], rot =[0,0,0,1], point_name='pointing_')
                return 'succ'
        else:
            d_l=np.linalg.norm(hum_pos[:2]-np.asarray((res.x_l,res.y_l)))
            d_r=np.linalg.norm(hum_pos[:2]-np.asarray((res.x_r,res.y_r)))
            if  d_l <threshold_bag_to_human  and d_l > d_r :
                tf_man.pub_static_tf(pos=[res.x_l, res.y_l,0], rot =[0,0,0,1], point_name='pointing_')
                if d_r <threshold_bag_to_human :
                    userdata.second_pointing=True
                    print("both hands, first right")
                    tf_man.pub_static_tf(pos=[res.x_r, res.y_r,0], rot =[0,0,0,1], point_name='pointing_2')
                return 'succ'
            if  d_r <threshold_bag_to_human  and d_r > d_l :
                tf_man.pub_static_tf(pos=[res.x_r, res.y_r,0], rot =[0,0,0,1], point_name='pointing_')
                if d_l <threshold_bag_to_human :
                    userdata.second_pointing=True
                    print("both hands, first left")
                    tf_man.pub_static_tf(pos=[res.x_l, res.y_l,0], rot =[0,0,0,1], point_name='pointing_2')
                return 'succ'
        ################################################################
        ########################################################
        talk('I did not find a pointing arm, I will try again')
        rospy.sleep(0.8)
        print ('no pointing ')
        self.tries -= 1
        return 'failed'

#########################################################################################################
class Scan_floor(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'], input_keys=['second_pointing'])
        self.tries = 0
        

    def execute(self, userdata):

        rospy.loginfo('STATE : Scan estimated pointing area')
        self.tries+=1 
        if self.tries==1:
            head.to_tf('pointing_')
            print("looking to TF")
                
        if self.tries==2 and userdata.second_pointing:
            head.to_tf('pointing_2')     
        if self.tries==3:
            self.tries=0
            return 'tries'
        rospy.sleep(3)
        

        try:
            img,obj = get_luggage_tf()
            save_image(img,name="segmentCarry")
            if obj:
                return 'succ'
            else:
                print(obj)
                talk('no Objects in area....')
                return 'failed'

        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")
        return 'failed'


###################################################################
class Scan_floor_deprecating(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0
        

    def execute(self, userdata):

        rospy.loginfo('STATE : Scan estimated pointing area')
        self.tries+=1  
        if self.tries==1:head.to_tf('pointing_')
        #if self.tries==2:head.to_tf('pointing_left')     
        if self.tries==3:
            self.tries=0
            talk("Let's do it again")
            return 'tries'
        
        
        ##### Segment and analyze   
        #img=rgbd.get_image()
        print ('got image for segmentation')
        ##### Segment and analyze
        rospy.sleep(1.5)
        prompt = "bag"     #put here favorite drink
        img=rgbd.get_image()
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        ros_image = bridge.cv2_to_imgmsg(img, encoding="bgr8")
        prompt_msg = String()
        prompt_msg.data = prompt
        

        # OLD BAREBONES SEGMENTATION####################        
        request= segmentation_server.request_class() 
        request.height.data=-0.05
        res=segmentation_server.call(request)
        img=bridge.imgmsg_to_cv2(res.im_out.image_msgs[0])
        save_image(img,name="segmentCarry")
        print (res.poses.data)
        ################################################
        _,_,theta=ransac_laser()
        print (theta,'\n\n\n\n')

        if len(res.poses.data)==0:
            talk('no Objects in area....')
            return 'failed'
        else:
            succ=seg_res_tf_pointing(res)   # la funcion seg res tf pointing traudce el response del servivio a una tf estatica. con el nombre pointing_right left
            return 'succ'
            
#########################################################################################################
class Pre_pickup(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=['succ', 'failed', 'tries'])
        self.target= 'bag'    
        
    def execute(self, userdata):
        rospy.loginfo(f'State : Pre PICKUP  {self.target} ')
        talk(f'Picking up luggage, please stay where you are')
        rospy.sleep(0.7)
        global target_object, pca_angle
        head.set_named_target('neutral')
        rospy.sleep(0.5)
        clear_octo_client()
        
        res = omni_base.move_d_to(0.55,self.target)
       

        gripper.open()

        

        if res:return 'succ'
        return 'failed'
        

        #return 0
        """
                                
                                
                                succ= arm.go(pickup_pose)
                                rospy.sleep(1.0)
                                
                        
                                if succ:
                                    
                                    succ = False
                                    
                                    while not succ:
                                        #trans,rot = tf_man.getTF(target_frame='static003_cracker_box', ref_frame='hand_palm_link')
                                        trans,_ = tf_man.getTF(target_frame=target_object, ref_frame='hand_palm_link')
                                        _,rot = tf_man.getTF(target_frame='base_link', ref_frame='map')
                        
                                        if type(trans) is bool:
                                            trans,_ = tf_man.getTF(target_frame='static029_plate', ref_frame='hand_palm_link')
                                            _,rot = tf_man.getTF(target_frame='base_link', ref_frame='map')
                                            print('no tf')
                                            return 'failed'
                                        if type(trans) is not bool:
                                            eX, eY, eZ = trans
                                            eY+=-0.05
                                            eT= tf.transformations.euler_from_quaternion(rot)[2] - 0.5*np.pi    #(known loc pickup change to line finder?)   NO! THERE ARE  ROUND TABLES !
                                            rospy.loginfo("Distance to goal: {:.2f}, {:.2f}, angle {:.2f}, target obj frame {}".format(eX, eY , eT,target_object))
                                            if abs(eX) < 0.03:
                                                eX = 0
                                            if abs(eY) < 0.01:
                                                eY = 0
                                            if abs(eT   ) < 0.05:
                                                eT = 0
                                            succ =  eX == 0 and eY == 0 and eT==0
                                                # grasp_base.tiny_move(velY=-0.4*trans[1], std_time=0.2, MAX_VEL=0.3)
                                            omni_base.tiny_move(velX=0.13*eX, velY=-0.4*eY, velT=-0.3*eT, std_time=0.2, MAX_VEL=0.3) #Pending test
                                    
                                    print(tf_man.getTF(ref_frame=target_object,target_frame='base_link'), 'tf obj base######################################')
                        
                                    
                                    return 'succ'"""
            

#########################################################################################################
"""class Pickup(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0
        self.target='object_0'
    def execute(self, userdata):
        rospy.loginfo('State :  PICKUP ')
        clear_octo_client()
        self.tries += 1
        target_object=self.target

        if self.tries >= 4:
            self.tries = 0
            return'tries'
        
        clear_octo_client()

        pickup_pose=[0.0,-1.4,0.0,-1.74, 0.0, 0.0]
        succ= arm.go(pickup_pose)
        
        succ = False
        
        brazo.move_hand_to_target(target_frame= target_object)
        gripper.close()

        arm.set_named_target('go')
        arm.go()
        res  = omni_base.move_base(known_location='living_room')
        succ=brazo.check_grasp()
        if res:
            return 'succ'
        else:
            return 'failed'
"""

#########################################################################################################
class Pickup_two(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0
        self.target='bag'    
    def execute(self, userdata):
        rospy.loginfo('State :  PICKUP ')
        clear_octo_client()
        self.tries += 1
        target_object=self.target

        if self.tries >= 3:
            self.tries = 0
            return'tries'
        
        clear_octo_client()

        _,rot= tf_man.getTF("base_link",ref_frame='map')
        original_rot=tf.transformations.euler_from_quaternion(rot)[2]
        target_object=self.target


        succ = False
                    
        while not succ:
            
            _,rot= tf_man.getTF("base_link",ref_frame='map')
            trans,_=tf_man.getTF(target_object,ref_frame="base_link")

            #trans
            eX, eY, eZ = trans
            
            eX+= -0.3
            eY+= -.1
            
            eT= tf.transformations.euler_from_quaternion(rot)[2] - original_rot #Original 
            print (eT)
            if eT > np.pi: eT=-2*np.pi+eT
            if eT < -np.pi: eT= 2*np.pi+eT
            rospy.loginfo("error: {:.2f}, {:.2f}, angle {:.2f}, target obj frame {}".format(eX, eY , eT,target_object))
            X, Y, Z = trans
            rospy.loginfo("Pose: {:.2f}, {:.2f}, angle {:.2f}, target obj frame {}".format(X, Y , eT,target_object))
            
            if abs(eX) <=0.25 :
                print ('here')
                eX = 0
            if abs(eY) <=0.05  :
                eY = 0
            if abs(eT   ) < 0.1:
                eT = 0
            succ =  eX == 0 and eY == 0 and eT==0

            
            omni_base.tiny_move( velX=0.2*+eX,velY=0.3*eY, velT=-eT,std_time=0.2, MAX_VEL=0.3) 

        rospy.sleep(2.0)
        clear_octo_client()
        rospy.sleep(1.0)
        print("MOVING ARM")    
        #_,_,theta=ransac_laser()
        #rint (theta)
        _,quat_r=tf_man.getTF('base_link')
        print(f'Quat base',np.rad2deg(tf.transformations.euler_from_quaternion(quat_r)[2]))
        ang_base=tf.transformations.euler_from_quaternion(quat_r)[2]
        _,quat_b=tf_man.getTF('bagpca')
        print(f'Quat bag' ,np.rad2deg(tf.transformations.euler_from_quaternion(quat_b)[2]))
        ang_bag=tf.transformations.euler_from_quaternion(quat_b)[2]
        

        #wrist_adjust =    max(theta-(0.5*np.pi), -1.57)
        wrist_adjust =    min(max((ang_base-ang_bag), -1.57) , 1.57)
        print ('theta, remvoe',wrist )
        #floor_pose=[0.05,-1.6,0.0,-1.41,0.0,0.0]

        floor_pose=[0.05,-1.6,0.0,-1.41,wrist_adjust,0.0]
        arm.set_joint_value_target(floor_pose)
        arm.go()
        rospy.sleep(1.0)
        print("closing hand")
        gripper.close(0.04)
        brazo.set_named_target('go')         
        head.to_tf('bag')
        rospy.sleep(3.0)
        succ = check_carry_bag()
        #succ=brazo.check_grasp()
        
        if succ:
            return 'succ'
        talk("I think I missed the object, I will retry")
        return 'failed'
        
#########################################################################################################
class Give_to_me(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed'])
        self.tries = 0
    def execute(self, userdata):
        rospy.loginfo("State: Give to me")
        brazo.set_named_target("neutral")
        talk("I failed, , please give the luggage to me")
        gripper.open()
        rospy.sleep(1.0)
        talk("In three...")
        rospy.sleep(0.4)
        talk("Two...")
        rospy.sleep(0.4)
        talk("One...")
        rospy.sleep(0.4)
        gripper.close(0.05)
        return 'succ'
        
#########################################################################################################
class Post_Pickup(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0    
    def execute(self, userdata):
        rospy.loginfo('State :  PICKUP ')
        clear_octo_client()
        self.tries += 1
        talk('Rotating to previous human location found')
        rospy.sleep(0.7)
        
        res = new_move_D_to(tf_name='human',d_x=15)
        #omni_base.move_d_to(target_distance= 1.0, target_link='human')
        if res:
            return 'succ'

        else:
            talk('Failed, retrying')
            rospy.sleep(0.8)
            return 'failed'

#########################################################################################################
class Deliver_Luggage(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed'])
        self.tries = 0
    def execute(self, userdata):
        rospy.loginfo("State : Deliver luggage to human")
        self.tries += 1
        deliver_position = [0.25, -0.32, -0.069, -1.25, 0.0]

        talk("Please take the luggage")
        brazo.set_joint_values(joint_values = deliver_position)
        talk("Releasing in three")
        rospy.sleep(0.7)
        talk("Two")
        rospy.sleep(0.7)
        talk("One")
        rospy.sleep(0.7)
        gripper.steady()
        talk('Push my hand when you have taken the bag')
        succ = wait_for_push_hand(100) # NOT GAZEBABLE
        if succ:return 'succ'
        # gripper.open()
        # talk("Thank you")
        # rospy.sleep(1)
        # talk("Closing Gripper. Please be sure your hand is out of the way")
        # rospy.sleep(2.5)
        # gripper.close(0.1)
        # return 'succ'

#########################################################################################################
class Return_Living_Room(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed'])
        self.tries = 0
    def execute(self, userdata):
        self.tries += 1
        talk("I will return to start location")
        res = omni_base.move_base(known_location='start_point', time_out=200)
        print(res)
        if res:
            talk("Task completed")
            return 'succ'
        else:
            talk('Navigation Failed, retrying')
            return 'failed'

#########################################################################################################

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
    sis = smach_ros.IntrospectionServer('SMACH_VIEW_SERVER', sm, '/SM_CARRY')
    sis.start()

    with sm:
        # State machine STICKLER
        smach.StateMachine.add("INITIAL",           Initial(),              transitions={'failed': 'INITIAL',           
                                                                                         'succ': 'WAIT_PUSH_HAND',   
                                                                                         #'succ': 'SCAN_FLOOR',   
                                                                                         'tries': 'END'})
        smach.StateMachine.add("WAIT_PUSH_HAND",    Wait_push_hand(),       transitions={'failed': 'WAIT_PUSH_HAND',    
                                                                                         'succ': 'FIND_HUMAN',       
                                                                                         'tries': 'END'})
        smach.StateMachine.add("GOTO_LIVING_ROOM",  Goto_living_room(),     transitions={'failed': 'GOTO_LIVING_ROOM',        
                                                                                         'succ': 'FIND_HUMAN',   
                                                                                         'tries': 'END'})  
        smach.StateMachine.add("FIND_HUMAN",         Find_human(),          transitions={'failed': 'FIND_HUMAN',        
                                                                                         'succ': 'SCAN_FLOOR',   
                                                                                         #'succ': 'GOTO_HUMAN',   
                                                                                         'tries': 'END'})
        smach.StateMachine.add("SCAN_FLOOR",         Scan_floor(),          transitions={'failed': 'SCAN_FLOOR',     
                                                                                         'succ': 'PRE_PICKUP',    
                                                                                         'tries': 'FIND_HUMAN'})   
        smach.StateMachine.add("PRE_PICKUP",        Pre_pickup(),           transitions={'failed': 'PRE_PICKUP',        
                                                                                         'succ': 'PICKUPTWO',  
                                                                                         'tries': 'END'})        
        """smach.StateMachine.add("PICKUP",            Pickup(),               transitions={'failed': 'PICKUP',        
                                                                                         'succ': 'GOTO_HUMAN',   
                                                                                         'tries': 'END'})      """  
        smach.StateMachine.add("PICKUPTWO",         Pickup_two(),           transitions={'failed': 'PICKUPTWO',        
                                                                                         'succ': 'POST_PICKUP',   
                                                                                         'tries': 'GIVE_TO_ME'})
        
        smach.StateMachine.add("GIVE_TO_ME",        Give_to_me(),                transitions={'failed': 'GIVE_TO_ME',
                                                                                         'succ': 'POST_PICKUP'})

        smach.StateMachine.add("POST_PICKUP",       Post_Pickup(),          transitions={'failed': 'POST_PICKUP',        
                                                                                         'succ': 'GOTO_HUMAN',   
                                                                                         'tries': 'END'})        
    
        smach.StateMachine.add("GOTO_HUMAN", SimpleActionState('follow_server', FollowAction), transitions={'aborted': 'GOTO_HUMAN',        
                                                                                                            'succeeded': 'DELIVER_LUGGAGE' ,   
                                                                                                            'preempted': 'END' })
        
        smach.StateMachine.add("DELIVER_LUGGAGE", Deliver_Luggage(),        transitions={'succ': 'RETURN_LIVING_ROOM', 
                                                                                         'failed': 'DELIVER_LUGGAGE'})
        
        smach.StateMachine.add("RETURN_LIVING_ROOM",  Return_Living_Room(), transitions={'succ': 'END', 
                                                                                         'failed':'RETURN_LIVING_ROOM'})


    outcome = sm.execute()

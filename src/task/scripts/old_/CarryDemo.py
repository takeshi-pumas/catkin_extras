#!/usr/bin/env python3
from smach_utils2 import *
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
        
        global arm

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
            talk('Starting Carry my luggage task')
            rospy.sleep(0.8)
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
            self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0
        self.point_img_pub = rospy.Publisher("/image_topic_name", String, queue_size=1)
        self.dist=2.5
    def execute(self, userdata):
        rospy.loginfo('State : Find_human')
        rospy.sleep(1.5)
        
        self.tries += 1
        if self.tries >= 4:
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
        rospy.sleep(1.0)
        talk('Two')
        rospy.sleep(1.0)
        talk('One')
        rospy.sleep(1.0)
        self.point_img_pub.publish(String())
        rospy.sleep(0.1)

        req = Point_detectorRequest()
        req.dist = self.dist
        req.removeBKG = True        # CAMBIAR SI SE QUIERE QUITAR EL FONDO (CHISMOSOS) O NO
        res=pointing_detect_server(req)
        print("RES",res.x_r,res.x_l,res.y_r,res.y_l)
        print(res.x_r+res.x_l,res.y_r+res.y_l)
        if (res.x_r+res.x_l) == -1 and  (res.y_r+res.y_l) == -1  :
            talk('I did not find a pointing arm, I will try again')
            rospy.sleep(0.8)
            print ('no pointing ')
            self.tries -= 1
            return 'failed'
        talk('Ok')
        rospy.sleep(0.8)
        if res.x_r ==-1: tf_man.pub_static_tf(pos=[res.x_l, res.y_l,0], rot =[0,0,0,1], point_name='pointing_')
        else: tf_man.pub_static_tf(pos=[res.x_r, res.y_r,0], rot =[0,0,0,1], point_name='pointing_')
        return 'succ'# There is now a "human TF with humans aprrox location(face)"

#########################################################################################################
class Scan_floor(smach.State):
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
            return 'tries'
        
        ##### Segment and analyze
        print ('got image for segmentation')
        ##### Segment and analyze
        rospy.sleep(1.5)
        request= segmentation_server.request_class() 
        request.height.data=-0.05
        res=segmentation_server.call(request)
        img=bridge.imgmsg_to_cv2(res.im_out.image_msgs[0])
        save_image(img,name="segmentCarry")
        print (res.poses.data)

        if len(res.poses.data)==0:
            talk('no objects in area....')
            return 'failed'
        else:
            succ=seg_res_tf_pointing(res)   # Cambia ya que depende de tf pointing_
            talk('Bag found')
            #succ=seg_res_tf(res)
            return 'succ'
            
#########################################################################################################
class Pre_pickup(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=['succ', 'failed', 'tries'])
        self.target= 'object_0'    
        
    def execute(self, userdata):
        rospy.loginfo(f'State : Pre PICKUP  {self.target} ')
        talk(f'Picking up luggage, please stay where you are')
        rospy.sleep(0.7)

        object_xyz,_=tf_man.getTF(self.target)
        print(f'[GOTOOBJECT] coords {object_xyz}')

        res = omni_base.move_d_to(0.7,self.target)
        global target_object, pca_angle
        head.set_named_target('neutral')
        rospy.sleep(0.5)
        clear_octo_client()

        gripper.open()
        pickup_pose=[0.0,-1.4,0.0,-1.74, 0.0, 0.0]
        succ= arm.go(pickup_pose)
        rospy.sleep(0.5)
        return 'succ'


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
            
            if abs(eX) <=0.05 :
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
        floor_pose=[0.05,-1.6,0.0,-1.41,0.0,0.0]
        arm.set_joint_value_target(floor_pose)
        arm.go()
        rospy.sleep(1.0)
        print("closing hand")
        gripper.close(0.04)
        rospy.sleep(3.0)
        succ=brazo.check_grasp()
        if succ:
            brazo.set_named_target('neutral')         
            return 'succ'
        talk (' I think I missed the object, I will retry ')
        return 'failed'
        
#########################################################################################################
class Give_to_me(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed'])
        self.tries = 0
    def execute(self, userdata):
        rospy.loginfo("State: Give to me")
        brazo.set_named_target("neutral")
        talk("I failed, give the lugg to me, please")
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
        gripper.open()
        talk("Thank you")
        return 'succ'

#########################################################################################################
class Finish(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed'])
        self.tries = 0
    def execute(self, userdata):
        self.tries += 1
        talk("Task completed")

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
    sis = smach_ros.IntrospectionServer('SMACH_VIEW_SERVER', sm, '/SM_STICKLER')
    sis.start()

    with sm:
        # State machine STICKLER
        
        
        smach.StateMachine.add("INITIAL",           Initial(),              transitions={'failed': 'INITIAL',           
                                                                                         'succ': 'WAIT_PUSH_HAND',   
                                                                                         'tries': 'END'})
        
        smach.StateMachine.add("WAIT_PUSH_HAND",    Wait_push_hand(),       transitions={'failed': 'WAIT_PUSH_HAND',    
                                                                                         'succ': 'FIND_HUMAN',       
                                                                                         'tries': 'END'})

        smach.StateMachine.add("FIND_HUMAN",         Find_human(),          transitions={'failed': 'FIND_HUMAN',        
                                                                                         'succ': 'SCAN_FLOOR',   
                                                                                         'tries': 'GOTO_LIVING_ROOM'})
        
        smach.StateMachine.add("GOTO_LIVING_ROOM",  Goto_living_room(),     transitions={'failed': 'GOTO_LIVING_ROOM',        
                                                                                         'succ': 'FIND_HUMAN',   
                                                                                         'tries': 'INITIAL'})  
        smach.StateMachine.add("SCAN_FLOOR",         Scan_floor(),          transitions={'failed': 'SCAN_FLOOR',     
                                                                                         'succ': 'PRE_PICKUP',    
                                                                                         'tries': 'END'})   
        smach.StateMachine.add("PRE_PICKUP",        Pre_pickup(),           transitions={'failed': 'PRE_PICKUP',        
                                                                                         'succ': 'PICKUPTWO',  
                                                                                         'tries': 'END'})        
        """smach.StateMachine.add("PICKUP",            Pickup(),               transitions={'failed': 'PICKUP',        
                                                                                         'succ': 'GOTO_HUMAN',   
                                                                                         'tries': 'END'})      """  
        smach.StateMachine.add("PICKUPTWO",         Pickup_two(),           transitions={'failed': 'GIVE_TO_ME',        
                                                                                         'succ': 'POST_PICKUP',   
                                                                                         'tries': 'END'})
        
        smach.StateMachine.add("GIVE_TO_ME",        Give_to_me(),                transitions={'failed': 'GIVE_TO_ME',
                                                                                         'succ': 'POST_PICKUP'})

        smach.StateMachine.add("POST_PICKUP",       Post_Pickup(),          transitions={'failed': 'POST_PICKUP',        
                                                                                         'succ': 'GOTO_HUMAN',   
                                                                                         'tries': 'END'})        
    
        smach.StateMachine.add("GOTO_HUMAN", SimpleActionState('follow_server', FollowAction), transitions={'aborted': 'GOTO_HUMAN',        
                                                                                                            'succeeded': 'DELIVER_LUGGAGE' ,   
                                                                                                            'preempted': 'END' })
        
        smach.StateMachine.add("DELIVER_LUGGAGE", Deliver_Luggage(),        transitions={'succ': 'Finish', 
                                                                                         'failed': 'DELIVER_LUGGAGE'})
        
        smach.StateMachine.add("Finish",  Finish(), transitions={'succ': 'END', 
                                                                                         'failed':'Finish'})


    outcome = sm.execute()

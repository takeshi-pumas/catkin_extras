#!/usr/bin/env python3
from smach_utils_pick_and_place import *


#-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
    ##### States definition #####
#Estado inicial de takeshi, neutral
class Initial(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ'],input_keys=['global_counter'])
        self.tries=0
    def execute(self,userdata):
        rospy.loginfo('STATE : robot neutral pose')
        self.tries+=1
        rospy.loginfo(f'Try:{self.tries} of 5 attempts') 
        if self.tries==3:
            return 'tries'
        # Set moveit initial values
        #eef_link = arm.get_end_effector_link()
        #scene.remove_attached_object(eef_link, name='box')
        #scene.remove_world_object('box')
        #gripper.steady()
        arm.set_named_target('go')
        arm.go()
        #arm.clear_pose_targets()
        clear_octo_client()
        return 'succ'

# --------------------------------------------------
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
        cabeza.set_named_target('neutral')
        brazo.set_named_target('go')
        gripper.steady()
        voice.talk('Gently... push my hand to begin')
        succ = wait_for_push_hand(100)

        if succ:
            return 'succ'
        else:
            return 'failed'

# --------------------------------------------------
class Goto_cassette_location(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'])
        self.tries=0
    def execute(self,userdata):
        rospy.loginfo('State : GOTO CASSETE... Go to known location ')
        self.tries+=1
        print('Try',self.tries,'of 5 attepmpts') 
        #if self.tries > 1:
        #    omni_base.tiny_move(velX=0.05,std_time=0.5, MAX_VEL=0.05)

        #clear_octo_client()
        print("Moviendose...")
        res = omni_base.move_base(known_location='cassette_demo',timeout=20)
        print(res)
        #AR_starter.call()

        rospy.sleep(0.8)
        if res:
            self.tries = 0
            return 'succ'
        else:
            voice.talk('Navigation Failed, retrying')
            return 'failed'
# --------------------------------------------------

class Goto_door(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'])
        self.tries=0
    def execute(self,userdata):
        rospy.loginfo('State : GOTO DOOR... Reset ')
        self.tries+=1
        print('Try',self.tries,'of 5 attepmpts') 
        #if self.tries > 1:
        #    omni_base.tiny_move(velX=0.05,std_time=0.5, MAX_VEL=0.05)

        #clear_octo_client()
        print("Moviendose...")
        res = omni_base.move_base(known_location='dining_room',timeout=20)
        print(res)
        #AR_starter.call()

        rospy.sleep(0.8)
        if res:
            self.tries = 0
            return 'succ'
        else:
            voice.talk('Navigation Failed, retrying')
            return 'failed'



# --------------------------------------------------
class Set_position(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'])
        self.tries=0
    def execute(self,userdata):
        global cassette_pose
        rospy.loginfo('State : Set position ')
        self.tries+=1
        print('Try',self.tries,'of 5 attepmpts') 
        floor_pose = [0.0,-2.47,0.0,0.86,-0.032,0.0]
        cassette_pose=[0.3, -0.87, 0.0, -0.75, -0.03, 0.0]
        print("Estableciendo pose hardcodeada....")
        arm.set_joint_value_target(cassette_pose)
        arm.plan()
        succ=arm.go()
        

        rospy.sleep(0.8)

        posMarker,rotMarker = tf_man.getTF(target_frame="ar_marker/201")
        if type(posMarker) is bool:
            print(posMarker)
            return 'tries'
        print (posMarker)
        tf_man.pub_static_tf(pos=posMarker, rot=rotMarker, point_name='marker', ref="map")

        pos,_ = tf_man.getTF(target_frame="marker", ref_frame="hand_palm_link")
        
        if type(pos) is bool:
            return 'tries'
        tf_man.pub_static_tf(pos=[pos[0] + 0.06, 0, 0.10], rot=[0, 0, 0, 1], point_name='goal', ref="hand_palm_link")
        trans, rot = tf_man.getTF(target_frame='goal', ref_frame='odom')
        
        if succ:
        #Get close to cassette
            #brazo.move_hand_to_target(target_frame = 'marker')
            
            self.tries=0
            return 'succ'
        else:
            return 'failed'

# --------------------------------------------------
class Pre_grasp_pose(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'])
        self.tries=0
    def execute(self,userdata):
        rospy.loginfo('State : Pre grasp pose')
        self.tries+=1
        print('Try',self.tries,'of 5 attepmpts') 
        if self.tries==1:
            talk("Getting close to the cassette")
            rospy.sleep(0.7)
        elif self.tries == 5:
            return 'failed'
        # State Pre grasp pose
        gripper.open()
        
        rospy.sleep(0.5)
        
        succ = False
        THRESHOLD = 0.01
        while not succ:
            trans,_ = tf_man.getTF(target_frame='marker', ref_frame='hand_palm_link')
            if type(trans) is not bool:
                _, eY, eX = trans
                rospy.loginfo("Distance to goal: {:.2f}, {:.2f}".format(eX, eY))
                if abs(eY) < THRESHOLD:
                    eY = 0
                if abs(eX) < THRESHOLD:
                    eX = 0
                succ =  eX == 0 and eY == 0
                    # grasp_base.tiny_move(velY=-0.4*trans[1], std_time=0.2, MAX_VEL=0.3)
                omni_base.tiny_move(velX=0.13*eX, velY=-0.4*eY, std_time=0.2, MAX_VEL=0.3) #Pending test
        return 'succ'

# --------------------------------------------------
class Grasp_pose(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'],input_keys=['global_counter'])
        self.tries=0
    def execute(self,userdata):
        rospy.loginfo('STATE : robot grasp pose')
        self.tries+=1
        print('Try',self.tries,'of 5 attepmpts') 
        if self.tries==4:
            return 'failed'
                # State grasp table
        try:
            calibrate_wrist.call()
        except:
            rospy.loginfo('Wrist not calibrated')
        gripper.open()
        # trans,rot= tf_man.getTF(target_frame='cassette', ref_frame='odom')
        rospy.sleep(0.5)
        # succ = False
        omni_base.tiny_move(velX=0.03,std_time=0.3,MAX_VEL=0.03)
        rospy.sleep(0.5)
        # succ = True
        return 'succ'

# --------------------------------------------------
class Grasp_table(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'],input_keys=['global_counter'])
        self.tries=0
    def execute(self,userdata):
        rospy.loginfo('STATE : Grasp from table')
        self.tries+=1
        print('Try',self.tries,'of 5 attepmpts') 

        if self.tries==5:
            return 'failed'

        # State post grasp pose
        clear_octo_client()
        # rospy.loginfo(f'fuerza inicial {FI}')
        rospy.sleep(0.5)
        # gripper.steady()
        gripper.close()
        # succ = False
        av= arm.get_current_joint_values()
        av[0] += 0.1
        arm.go(av)
        
        rospy.sleep(0.8)
        force = wrist.get_force()
        force = np.array(force)
        weight = force[0]
        rospy.loginfo("Weight detected of {:.3f} Newtons".format(weight))
        if  weight >  0.01:
            talk('I have the cassette')
            rospy.sleep(0.7)
            return 'succ'
        else:
            trans, _ = tf_man.getTF(target_frame= 'hand_r_finger_tip_frame', ref_frame= 'hand_l_finger_tip_frame')
            t = np.array(trans)
            dist = np.linalg.norm(t)
            if dist > 0.005:
                talk('succeed')
                rospy.sleep(0.7)
                return 'succ'
            else:
                gripper.open()
                rospy.sleep(0.1)
                #trans[2] -= 0.03
                #pose_goal = set_pose_goal(pos = trans, rot=rot)
                av=arm.get_current_joint_values()
                av[0]+=-0.01
                arm.set_joint_value_target(av)
                
                talk('I will try again')
                arm.go()
                rospy.sleep(0.7)
                return 'tries'

# --------------------------------------------------
class Post_grasp_pose(smach.State):###example of a state definition.
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'])
        self.tries=0
    def execute(self,userdata):
        rospy.loginfo('STATE : Post grasp pose')
        self.tries+=1
        print('Try',self.tries,'of 5 attepmpts') 
        
        if self.tries==3:
            return 'tries'
        omni_base.tiny_move(velX=-0.1,std_time=1.5, MAX_VEL=0.1)
        rospy.sleep(0.7)
        arm.set_named_target('go')
        succ = arm.go()
        cabeza.set_named_target('neutral')
        #head.go()
        if succ:
            talk('Going to  the player ')
            rospy.sleep(0.7)
            return 'succ'
        else:
            return 'tries'

# --------------------------------------------------
class Attach_object(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'],input_keys=['global_counter'])
        self.tries=0
    def execute(self,userdata):
        rospy.loginfo('STATE : Attach object')
        self.tries+=1
        print('Try',self.tries,'of 5 attepmpts') 
        
        if self.tries==3:
            return 'tries'
        # State Delete objects
        eef_link = arm.get_end_effector_link()
        # Adding objects to planning scene
        box_pose = PoseStamped()
        box_pose.header.frame_id = "hand_palm_link"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.z =  0.08  # below the panda_hand frame
        box_name = "box"
        scene.add_box(box_name, box_pose, size=(0.13, 0.05, 0.13))
        rospy.sleep(0.7)
        ##attaching object to the gripper
        grasping_group = "gripper"
        touch_links = robot.get_link_names(group=grasping_group)
        scene.attach_box(eef_link, box_name, touch_links=touch_links)
        # grasp_base.tiny_move(velX = -0.05, std_time=1.0, MAX_VEL=0.05)
        # arm.set_named_target('go')
        # arm.go()
        trans, _= tf_man.getTF(target_frame='marker', ref_frame='base_link')
        hcp = cabeza.relative(trans[0],trans[1],trans[2])
        #head.set_joint_value_target(hcp)
        #succ = head.go()
        #if succ:
        return 'succ'
        #else:
        #    return 'tries'

# --------------------------------------------------
class Delete_objects(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'],input_keys=['global_counter'])
        self.tries=0
    def execute(self,userdata):
        rospy.loginfo('STATE : Delete objects')
        self.tries+=1
        print('Try',self.tries,'of 5 attepmpts') 
        
        if self.tries==3:
            return 'tries'
        # State Delete objects
        ##Detaching objects
        eef_link = arm.get_end_effector_link()
        scene.remove_attached_object(eef_link, name='box')

        ##Remove objects
        scene.remove_world_object('box')
        gripper.open()
        omni_base.tiny_move(velX = -0.05, std_time=1.0, MAX_VEL=0.05)
        arm.set_named_target('go')
        arm.go()
        return 'succ'

# --------------------------------------------------

class Goto_player(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'],input_keys=['global_counter'])
        self.tries=0
    def execute(self,userdata):
        rospy.loginfo('STATE : GOTO player location')
        self.tries+=1
        print('Try',self.tries,'of 5 attepmpts') 
        
        if self.tries==3:
            return 'failed'
       
        status = 1
        print("Moviendose a known location de player...")
        res = omni_base.move_base(known_location='player_demo',timeout=20)
        if not(res) and self.tries<6:
            return 'tries'
        
        self.tries=0
        return 'succ'



# --------------------------------------------------
class Embudo_search(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'],input_keys=['global_counter'])
        self.tries=0
    def execute(self,userdata):
        rospy.loginfo('STATE : embudo searching')
        self.tries+=1
        print('Try',self.tries,'of 5 attepmpts') 
        trans,rot = tf_man.getTF(target_frame='base_link')

        #TODO CORRECT ORIENTATION?
        print (tf.transformations.euler_from_quaternion(rot)[2],'Orientation post nav')
        if self.tries==2:
            hp=cabeza.get_joint_values()
            hp[1]-=0.4    
            cabeza.set_joint_values(hp)    
        
        talk("Looking For the Player")
        rospy.sleep(0.8)

        
        rospy.sleep(0.9)
        
        succ = False
        flag = 1
        while not succ:
            trans,rot = tf_man.getTF(target_frame='ar_marker/6')

            if type(trans) is not bool:
                cabeza.set_named_target('neutral')

                tf_man.pub_static_tf(pos=trans, rot=rot, point_name='embudo_loc')
                rospy.sleep(0.8)
                succ = True
                self.tries=0
                return 'succ'
          
            else:

                return 'tries'


# --------------------------------------------------
class Embudo_place(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'],input_keys=['global_counter'])
        self.tries=0
    def execute(self,userdata):
        rospy.loginfo('State : placing arm in place cassete pos 6')
        self.tries+=1
        print('Try',self.tries,'of 5 attepmpts') 
        clear_octo_client()
        
        #player_pose=[0.42, -0.87, 0.0, -0.75, -0.03, 0.0]
        player_pose=[0.46, -0.87, 0.0, -0.75, 0.0, 0.0]
        arm.set_joint_value_target(player_pose)
        arm.go()

        
        succ = False
        THRESHOLD = 0.01
        while not succ:
            trans,_ = tf_man.getTF(target_frame='embudo_loc', ref_frame='hand_palm_link')
            print (trans)
            if type(trans) is not bool:
                _, eY, eX = trans
                rospy.loginfo("Distance to goal: {:.2f}, {:.2f}".format(eX, eY))
                if abs(eY) < THRESHOLD:
                    eY = 0
                if abs(eX) < THRESHOLD:
                    eX = 0
                succ =  eX == 0 and eY == 0
                    # grasp_base.tiny_move(velY=-0.4*trans[1], std_time=0.2, MAX_VEL=0.3)
                omni_base.tiny_move(velX=0.15*eX, velY=-0.4*eY, std_time=0.2, MAX_VEL=0.3) #Pending test
        


        av=arm.get_current_joint_values()
        av[0]+=-0.061
        av[3] = -1
        av[4]= 1.57
        arm.go(av)
        talk ('releasing')
        gripper.open()
        rospy.sleep(1.0)
        gripper.close()
        av=arm.get_current_joint_values()
        av[0]+=0.071
        av[4]= 0.0
        arm.go(av)
        return 'succ'
# --------------------------------------------------

class Post_embudo_place(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'],input_keys=['global_counter'])
        self.tries=0
    def execute(self,userdata):
        rospy.loginfo('State : Post placing Player')
        self.tries+=1
        

        gripper.open()
        rospy.sleep(0.7)
        omni_base.tiny_move(velX=-0.1,std_time=1.5, MAX_VEL=0.1)
        arm.set_named_target('go')
        succ = arm.go()
        cabeza.set_named_target('neutral')
        gripper.close()


        talk ('Demo Finished, Thanks for watching, Have a nice day')
        return 'succ'
        
# --------------------------------------------------
        



#==============================================================================
#Entry point    
if __name__== '__main__':
    print("TAKESHI STATE MACHINE...")

    sm = smach.StateMachine(outcomes = ['END'])     #State machine, final state "END"
    sis = smach_ros.IntrospectionServer('SMACH_VIEW_SERVER', sm, '/SM_CASSETTE_DEMO')
    sis.start()
    with sm:

        smach.StateMachine.add("INITIAL",           Initial(),                  transitions = {'succ':'WAIT_PUSH_HAND'}) 
        smach.StateMachine.add("WAIT_PUSH_HAND",    Wait_push_hand(),           transitions = {'failed':'WAIT_PUSH_HAND',   'succ':'GOTO_CASSETTE',     'tries':'END'}) 
        smach.StateMachine.add("GOTO_CASSETTE",     Goto_cassette_location(),   transitions = {'failed':'GOTO_CASSETTE',    'succ':'SET_POSITION',      'tries':'END'}) 
        smach.StateMachine.add("SET_POSITION",      Set_position(),             transitions = {'failed':'SET_POSITION',     'succ':'PRE_GRASP_POSE',    'tries':'SET_POSITION'})
        smach.StateMachine.add("PRE_GRASP_POSE",    Pre_grasp_pose(),           transitions = {'failed':'END',              'succ':'GRASP_POSE',        'tries':'PRE_GRASP_POSE'}) 
        smach.StateMachine.add("GRASP_POSE",        Grasp_pose(),               transitions = {'failed':'END',              'succ':'GRASP_TABLE',       'tries':'GRASP_POSE'}) 
        smach.StateMachine.add("GRASP_TABLE",       Grasp_table(),              transitions = {'failed':'GOTO_DOOR',        'succ':'ATTACH_OBJECT',     'tries':'GRASP_TABLE'})
        smach.StateMachine.add("ATTACH_OBJECT",     Attach_object(),            transitions = {'failed':'POST_GRASP_POSE',  'succ':'POST_GRASP_POSE',   'tries':'POST_GRASP_POSE'})
        smach.StateMachine.add("DELETE_OBJECTS",    Delete_objects(),           transitions = {'failed':'END',              'succ':'END',               'tries':'END'})
        smach.StateMachine.add("GOTO_DOOR",         Goto_door(),                transitions = {'failed':'GOTO_DOOR',        'succ':'GOTO_CASSETTE',     'tries':'END'}) 
        smach.StateMachine.add("POST_GRASP_POSE",   Post_grasp_pose(),          transitions = {'failed':'END',              'succ':'GOTO_PLAYER',       'tries':'POST_GRASP_POSE'})
        smach.StateMachine.add("GOTO_PLAYER",       Goto_player(),              transitions = {'failed':'GOTO_PLAYER',      'succ':'EMBUDO_LOC',        'tries':'GOTO_PLAYER'})
        smach.StateMachine.add("EMBUDO_LOC",        Embudo_search(),            transitions = {'failed':'EMBUDO_LOC',       'succ':'EMBUDO_PLACE',      'tries':'EMBUDO_LOC'})
        smach.StateMachine.add("EMBUDO_PLACE",      Embudo_place(),             transitions = {'failed':'EMBUDO_PLACE',     'succ':'POST_EMBUDO_PLACE', 'tries':'EMBUDO_PLACE'})
        smach.StateMachine.add("POST_EMBUDO_PLACE", Post_embudo_place(),        transitions = {'failed':'POST_EMBUDO_PLACE','succ':'END',               'tries':'POST_EMBUDO_PLACE'})        

    outcome = sm.execute()

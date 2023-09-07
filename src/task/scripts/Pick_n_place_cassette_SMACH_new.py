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
        #arm.clear_pose_targets()
        #clear_octo_client()
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
        rospy.loginfo('State : Set position ')
        self.tries+=1
        print('Try',self.tries,'of 5 attepmpts') 
        #cabeza.relative(1,0,0.7)
        #TO DO: decide if use Pre defined pose (brazo) or to compute pose (moveit)
        #brazo.set_joint_values(joint_values = [0.0, 0.0, -1.6, -1.6, 0.0])
        #tf_man.getTF(target_frame = "hand_palm_link")
        #if self.tries==3:
            # SI NO LOGRA PLANEAR, SE MUEVE UN POQUITO (CON EL CONTROL SE VIO QUE YA LOGRA PLANEAR 
            #                                           AL MOVERLE UN POQUITO ADELANTE O A UN LADO)
        #    omni_base.tiny_move(velX = -0.02, std_time=1.0, MAX_VEL=0.04)

        brazo.set_named_target('neutral')
        rospy.sleep(0.8)

        posMarker,rotMarker = tf_man.getTF(target_frame="ar_marker/201")
        if posMarker[0]==None:
            print(posMarker)
            return 'tries'
        
        tf_man.pub_static_tf(pos=posMarker, rot=rotMarker, point_name='marker', ref="map")

        pos,_ = tf_man.getTF(target_frame="marker", ref_frame="hand_palm_link")
        tf_man.pub_static_tf(pos=[pos[0] + 0.06, 0, 0.10], rot=[0, 0, 0, 1], point_name='goal', ref="hand_palm_link")
        trans, rot = tf_man.getTF(target_frame='goal', ref_frame='odom')
        pose_goal = Pose()
        pos = Point(*trans)
        quat = Quaternion(*rot)
        pose_goal.orientation = quat
        pose_goal.position = pos

        arm.set_pose_target(pose_goal)
        succ = arm.go()

        if succ:
        #Get close to cassette
            brazo.move_hand_to_target(target_frame = 'marker')
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
            talk("I will reach the cassette")
            rospy.sleep(0.7)
        elif self.tries == 5:
            return 'failed'
        # State Pre grasp pose
        gripper.open()
        #arm.set_named_target('neutral')
        #arm.go()
        #Set pose
        #trans,_ = tf_man.getTF(target_frame='marker', ref_frame='hand_palm_link')
        #tf_man.pub_static_tf(point_name='goal_pose', pos=[trans[0]+0.09, 0.0, 0.11], ref='hand_palm_link')
        #pos,_ = tf_man.getTF(target_frame='hand_palm_link', ref_frame='odom')
        #if (type(trans) is bool) and (type(pos) is bool) :
        #    return 'tries'
        rospy.sleep(0.5)
        #pos, rot = tf_man.getTF(target_frame='goal_pose', ref_frame='odom')
        #if type(pos) is not bool:
        #    pose_goal = set_pose_goal(pos=pos, rot=rot)
        #    arm.set_start_state_to_current_state() 
        #    arm.set_pose_target(pose_goal)
        #    succ, plan, _, _ = arm.plan()
        #    if succ:
        #        arm.execute(plan)
        #    else:
        #        rospy.loginfo('I could not plan to goal pose')
        #        return 'tries'
        #else:
        #    return 'tries'
        #Align along X and Y axis
        succ = False
        THRESHOLD = 0.01
        while not succ:
            trans,_ = tf_man.getTF(target_frame='marker', ref_frame='hand_palm_link')
            if type(trans) is not bool:
                _, eY, eX = trans
                rospy.loginfo("Distance to goal: {:.2f}, {:.2f}".format(eX, eY))
                if abs(eY) < THRESHOLD:
                    eY = 0
                if abs(eX) < THRESHOLD-0.0003:
                    eX = 0
                succ =  eX == 0 and eY == 0
                    # grasp_base.tiny_move(velY=-0.4*trans[1], std_time=0.2, MAX_VEL=0.3)
                omni_base.tiny_move(velX=0.3*eX, velY=-0.4*eY, std_time=0.2, MAX_VEL=0.3) #Pending test
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
        trans, rot = tf_man.getTF(target_frame='hand_palm_link', ref_frame='odom') 
        trans[2] += 0.03
        # while not succ:
        pose_goal = set_pose_goal(pos = trans, rot =rot)
        # wb.set_start_state_to_current_state()
        # wb.set_pose_target(pose_goal)
        arm.set_start_state_to_current_state()
        arm.set_pose_target(pose_goal)
        # succ, plan, _, _ = wb.plan()
        succ, plan, _, _ = arm.plan()

        if succ:
            # wb.execute(plan)
            arm.execute(plan)
        else:
            return 'tries'
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
                trans[2] -= 0.03
                pose_goal = set_pose_goal(pos = trans, rot=rot)
                arm.set_start_state_to_current_state()
                arm.set_pose_target(pose_goal)
                succ, plan, _, _ = arm.plan()
                if succ:
                    arm.execute(plan)
                talk('I will try again')
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
            talk('Now I will find the player')
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
class Forward_shift(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'],input_keys=['global_counter'])
        self.tries=0
    def execute(self,userdata):
        rospy.loginfo('STATE : Forward shift')
        self.tries+=1
        print('Try',self.tries,'of 5 attepmpts') 
        
        if self.tries==3:
            return 'tries'
# State forward shift
        ##Detaching objects
        # scene.remove_attached_object(eef_link, name=box_name)

        ##Remove objects
        # scene.remove_world_object(box_name)
        omni_base.tiny_move(velX = 0.02, std_time=0.5, MAX_VEL=0.03)
        succ = True
        if succ:
            return 'succ'
        else:
            return 'failed'

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
        # State Player AR search
        # status = goto('player_demo')
        status = 1
        print("Moviendose a known location de player...")
        res = omni_base.move_base(known_location='player_demo',timeout=20)
        if not(res) and self.tries<6:
            return 'tries'
        
        self.tries=0
        return 'succ'


# --------------------------------------------------
class Player_search(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'],input_keys=['global_counter'])
        self.tries=0
    def execute(self,userdata):
        #rospy.loginfo('STATE : Player search')
        self.tries+=1
        print('Try',self.tries,'of 5 attepmpts') 
        
        if self.tries==2:
            hp=cabeza.get_joint_values()
            hp[0]-=0.6    
            cabeza.set_joint_values(hp)    
        # State Player AR search
        # status = goto('player_demo')
        #status = 1
        #print("Moviendose a known location de player...")
        #res = omni_base.move_base(known_location='player_demo',timeout=20)
        #if not(res) and self.tries<6:
        #    return 'tries'
        # while status !=3:
            # status = NS.get_status()
            # print(status)
            # rospy.sleep(0.5)
        # return 'succ'
        talk("Finding player")
        rospy.sleep(0.8)

        try:
            AR_starter.call()
            clear_octo_client()
        except:
            rospy.loginfo('Cant clear octomap')
        rospy.sleep(0.9)
        #hcp = cabeza.relative(1,0,0.7)
        #head.set_joint_value_target(hcp)
        #head.go()
        succ = False
        flag = 1
        while not succ:
            trans,rot = tf_man.getTF(target_frame='ar_marker/7')

            if type(trans) is not bool:
                cabeza.set_named_target('neutral')

                tf_man.pub_static_tf(pos=trans, rot=rot, point_name='player')
                rospy.sleep(0.8)
                succ = True
                self.tries=0
                return 'succ'
            #else:
            #    gazeY = -0.5 
            #    if flag == 1:
            #        flag += 1
            #    elif flag == 2:
            #        gazeY = 0.5
            #        flag += 1
            #    else:
            #        head.set_named_target('neutral')
            #        head.go()
            #        talk('I did not find any marker, I will try again') 
            #        return 'tries'
                #hcp = cabeza.relative(0.5,gazeY,0.7)
                #head.set_joint_value_target(hcp)
                #head.go()
            #    rospy.sleep(0.3)
            else:

                return 'tries'


# --------------------------------------------------
class Player_alignment(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'],input_keys=['global_counter'])
        self.tries=0
    def execute(self,userdata):
        rospy.loginfo('STATE : player alignment')
        self.tries+=1
        rospy.loginfo(f'Try:{self.tries} of 5 attempts') 
        if self.tries==3:
            return 'tries'
        #Player alignment
        arm.set_named_target('neutral')
        arm.go()
        try:
            acp = arm.get_current_joint_values()
            acp[4] = 1.60
            arm.set_joint_value_target(acp)
            arm.go()
        except:
            return 'tries'
        succ = False
        THRESHOLD = 0.01
        ret=''
        while not succ:
            trans,rot = tf_man.getTF(target_frame='player', ref_frame='hand_palm_link')
            print("ROT player: ",rot)
            if rot==False:
                ret='tries'
                break
            euler = tf.transformations.euler_from_quaternion(rot)
            theta = euler[1] 
            if type(trans) is not bool:
                eY, _, eX = trans
                eY = -eY 
                eX -= 0.40
                eT = theta
                rospy.loginfo("Distance to goal: {:.3f}, {:.3f}".format(eX, eY))
                if abs(eY) < THRESHOLD:
                    eY = 0
                if abs(eX) < THRESHOLD:
                    eX = 0
                if abs(eT) < THRESHOLD:
                    # talk("I am aligned")
                    eT=0
                succ =  eX == 0 and eY == 0
#                 rospy.sleep(1.0)
                omni_base.tiny_move(velX=0.2*eX, velY=0.4*eY, velT=-0.4*eT, std_time=0.2, MAX_VEL=0.3) #Pending test
                ret='succ'
        return ret

# --------------------------------------------------
class Pre_place_pose(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'],input_keys=['global_counter'])
        self.tries=0
    def execute(self,userdata):
        rospy.loginfo('STATE : Pre place pose')
        self.tries+=1
        rospy.loginfo(f'Try:{self.tries} of 5 attepmpts') 
        # if self.tries > 1:
        clear_octo_client()
        if self.tries==5:
            return 'failed'
        cabeza.set_named_target('neutral')
        #head.go()
        arm.set_goal_tolerance(0.001)

        pos, rot = tf_man.getTF(target_frame='hand_palm_link', ref_frame='odom')
        trans,_ = tf_man.getTF(target_frame='player', ref_frame='hand_palm_link')
        # sube un poco el brazo?????????????
        pos[2]+=(-trans[1]+0.017)
        tf_man.pub_static_tf(point_name='goal',pos=pos, rot=rot, ref='odom')
        tf_man.pub_static_tf(point_name='goal_pose', pos=[0,0,0.04], ref='goal')
        pos, rot = tf_man.getTF(target_frame='goal_pose', ref_frame='odom')
        pose_goal = set_pose_goal(pos=pos, rot = rot)
        arm.set_start_state_to_current_state()
        arm.set_pose_target(pose_goal)
        succ,plan,_,_=arm.plan()
        if succ:
            arm.execute(plan)
            return 'succ'
        else:
            return 'tries'

# --------------------------------------------------
class Get_player_edge(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'],input_keys=['global_counter'])
        self.tries=0
    def execute(self,userdata):
        rospy.loginfo('STATE : robot neutral pose')
        self.tries+=1
        rospy.loginfo(f'Try:{self.tries} of 5 attepmpts') 
        if self.tries==3:
            return 'tries'
        succ = False
        while not succ:
            force = wrist.get_force()
            if force[2] <= 0.0:
                omni_base.tiny_move(velX=0.03, std_time=0.01)
            else:
                vel = -0.01
                omni_base.tiny_move(velX=vel, std_time=0.1)
                p,r = tf_man.getTF(target_frame = 'hand_r_finger_tip_frame')
                tf_man.pub_static_tf(point_name='goal_pose',pos=p, rot=r, ref='map')
                succ = True
                self.tries=0
        return 'succ'

# --------------------------------------------------
class Pre_place_cassette(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'],input_keys=['global_counter'])
        self.tries=0
    def execute(self,userdata):
        rospy.loginfo('STATE : Pre place cassette ')
        self.tries+=1
        rospy.loginfo(f'Try:{self.tries} of 5 attepmpts') 
        if self.tries==3:
            return 'tries'
        clear_octo_client()
        succ = False
        THRESHOLD = 0.0095
        while not succ:
            trans,_ = tf_man.getTF(target_frame='player', ref_frame='hand_palm_link')
            if type(trans) is not bool:
                eY, _, eX = trans
                eY = -eY - 0.10
                eX -= 0.17
                rospy.loginfo("Distance to goal: {:.3f}, {:.3f}".format(eX, eY))
                print("Moving right until some THRESHOLD")
                if abs(eY) < THRESHOLD:
                    eY = 0
                if abs(eX) < THRESHOLD:
                    eX = 0
                succ =  eX == 0 and eY == 0
#                 rospy.sleep(1.0)
                omni_base.tiny_move(velX=0.2*eX, velY=0.45*eY, std_time=0.25, MAX_VEL=0.3) #Pending test
        return 'succ'

# --------------------------------------------------
class Place_cassette(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'],input_keys=['global_counter'])
        self.tries=0
    def execute(self,userdata):
        rospy.loginfo('STATE : placing cassette inside')
        self.tries+=1
        rospy.loginfo(f'Try:{self.tries} of 5 attepmpts') 
        if self.tries==3:
            return 'tries'
        succ = False
        e1= 1.0
        n1 = 0.015 * 0.9 #Amplificacion de la fuerza frontal
        n2 = 0.02       #Amplificacion de la fuerza lateral
        c = 1.05        #
        THRESHOLD = 0.001
        TIMEOUT = 8
        start_time = rospy.Time().now().to_sec()
        while not succ:
            d, _ = tf_man.getTF(target_frame='player', ref_frame='hand_r_finger_tip_frame')
            d_GP,_ = tf_man.getTF(target_frame='goal_pose', ref_frame='hand_r_finger_tip_frame')
            force = wrist.get_force()
            dist = np.array([c*(d[1]),0.0])
            if d_GP[1] >= 0.0:
                n2 = 0.025
            else:
                n2 = 0.001
            f = np.array([n2*force[2],-n1*force[0]])
            speed = e1*dist-f
            #print(f'Distance to AR x:{d[0]}, z:{d[1]}')
            succ =  abs(dist[0]) < THRESHOLD
            if not succ:
                omni_base.tiny_move(velX = speed[0], velY = speed[1], std_time = 0.02, MAX_VEL = 0.03)
            if rospy.Time().now().to_sec() - start_time > TIMEOUT:
                return 'tries'
        return 'succ'

# --------------------------------------------------
class Push_cassette(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'],input_keys=['global_counter'])
        self.tries=0
    def execute(self,userdata):
        rospy.loginfo('STATE : robot neutral pose')
        self.tries+=1
        rospy.loginfo(f'Try:{self.tries} of 5 attepmpts') 
        if self.tries==3:
            return 'tries'
        gripper.steady()
        succ = False
        THRESHOLD = 0.07
        #Put hand behind cassette
        while not succ:
            d, _ = tf_man.getTF(target_frame='player', ref_frame='hand_r_finger_tip_frame')
            succ = d[1] > THRESHOLD
            if not succ:
                omni_base.tiny_move(velX = -0.04,std_time=0.2)
        gripper.close()
        succ = False
        THRESHOLD = 0.04
        while not succ:
            d, _ = tf_man.getTF(target_frame='player', ref_frame='hand_r_finger_tip_frame')
            succ = d[1] < THRESHOLD
            if not succ:
                omni_base.tiny_move(velX = 0.04,std_time=0.2)
        gripper.steady()
        return 'succ'

#==============================================================================
#Entry point    
if __name__== '__main__':
    print("TAKESHI STATE MACHINE...")
    sm = smach.StateMachine(outcomes = ['END'])     #State machine, final state "END"

    with sm:
        smach.StateMachine.add("INITIAL",           Initial(),          transitions = {'succ':'GOTO_CASSETTE'}) 
        smach.StateMachine.add("WAIT_PUSH_HAND",    Wait_push_hand(),   transitions = {'failed':'WAIT_PUSH_HAND','succ':'GOTO_CASSETTE',    'tries':'END'}) 
        
        smach.StateMachine.add("GOTO_CASSETTE",     Goto_cassette_location(),   transitions = {'failed':'GOTO_CASSETTE','succ':'SET_POSITION',      'tries':'END'}) 
        smach.StateMachine.add("SET_POSITION",      Set_position(),     transitions = {'failed':'SET_POSITION',         'succ':'PRE_GRASP_POSE',    'tries':'SET_POSITION'})

        smach.StateMachine.add("PRE_GRASP_POSE",    Pre_grasp_pose(),   transitions = {'failed':'END',                  'succ':'GRASP_POSE',        'tries':'PRE_GRASP_POSE'}) 
        smach.StateMachine.add("FORWARD_SHIFT",     Forward_shift(),    transitions = {'failed':'FORWARD_SHIFT',        'succ':'GRASP_TABLE',       'tries':'FORWARD_SHIFT'}) 
        smach.StateMachine.add("GRASP_POSE",        Grasp_pose(),       transitions = {'failed':'END',                  'succ':'GRASP_TABLE',       'tries':'GRASP_POSE'}) 
        smach.StateMachine.add("GRASP_TABLE",       Grasp_table(),      transitions = {'failed':'DELETE_OBJECTS',       'succ':'ATTACH_OBJECT',     'tries':'GRASP_TABLE'})
        smach.StateMachine.add("ATTACH_OBJECT",     Attach_object(),    transitions = {'failed':'POST_GRASP_POSE',      'succ':'POST_GRASP_POSE',   'tries':'POST_GRASP_POSE'})
        smach.StateMachine.add("DELETE_OBJECTS",    Delete_objects(),   transitions = {'failed':'END',                  'succ':'END',               'tries':'END'})
        smach.StateMachine.add("POST_GRASP_POSE",   Post_grasp_pose(),  transitions = {'failed':'END',                  'succ':'GOTO_PLAYER',     'tries':'POST_GRASP_POSE'})

        smach.StateMachine.add("GOTO_PLAYER",       Goto_player(),      transitions = {'failed':'GOTO_PLAYER',          'succ':'PLAYER_SEARCH',  'tries':'GOTO_PLAYER'})
        smach.StateMachine.add("PLAYER_SEARCH",     Player_search(),    transitions = {'failed':'PLAYER_SEARCH',        'succ':'PLAYER_ALIGNMENT',  'tries':'PLAYER_SEARCH'})
        
        smach.StateMachine.add("PLAYER_ALIGNMENT",  Player_alignment(), transitions = {'failed':'PLAYER_ALIGNMENT',     'succ':'PRE_PLACE_POSE',    'tries':'PLAYER_ALIGNMENT'})
        smach.StateMachine.add("PRE_PLACE_POSE",    Pre_place_pose(),   transitions = {'failed':'PLAYER_SEARCH',        'succ':'GET_PLAYER_EDGE',   'tries':'PRE_PLACE_POSE'})

        smach.StateMachine.add("GET_PLAYER_EDGE",   Get_player_edge(),  transitions = {'failed':'GET_PLAYER_EDGE',      'succ':'PRE_PLACE_CASSETTE','tries':'GET_PLAYER_EDGE'})
        smach.StateMachine.add("PRE_PLACE_CASSETTE",Pre_place_cassette(),transitions = {'failed':'PRE_PLACE_CASSETTE',  'succ':'PLACE_CASSETTE',    'tries':'PRE_PLACE_CASSETTE'})
        smach.StateMachine.add("PLACE_CASSETTE",    Place_cassette(),   transitions = {'failed':'PLACE_CASSETTE',       'succ':'PUSH_CASSETTE',     'tries':'END'})
        smach.StateMachine.add("PUSH_CASSETTE",     Push_cassette(),    transitions = {'failed':'PUSH_CASSETTE',        'succ':'END',               'tries':'PUSH_CASSETTE'})
    outcome = sm.execute()

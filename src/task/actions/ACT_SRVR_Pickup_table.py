#!/usr/bin/env python3

from utils_action import *
from smach_ros import ActionServerWrapper
from action_pickup_floor.msg import PickUpAction
##### Define state INITIAL #####

# --------------------------------------------------
class Initial(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):
        global arm
        rospy.loginfo('STATE : INITIAL')
        print('Robot neutral pose')
        self.tries += 1
        print(f'Try {self.tries} of 5 attempts')
        if self.tries == 3:
            return 'tries'
        
        arm = moveit_commander.MoveGroupCommander('arm')
        head.set_named_target('neutral')

        rospy.sleep(0.8)
        arm.set_named_target('go')
        arm.go()
        rospy.sleep(0.3)



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
            self.tries=0
            return 'tries'
        talk('Gently... push my hand to continue')
        succ = wait_for_push_hand(100)

        if succ:
            return 'succ'
        else:
            return 'failed'
#########################################################################################################
class Find_object(smach.State):  # ADD KNONW LOCATION DOOR
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):

        rospy.loginfo('STATE : Segment and tfing object' )

        print(f'Try {self.tries} of 3 attempts')
        self.tries += 1
        if self.tries == 4:
            self.tries=0
            return 'tries'


        request= segmentation_server.request_class() 
        request.height.data=-1.0 # AUTODETECT PLANE HEIGHT
        #request.height.data=0.41 # AUTODETECT PLANE HEIGHT
        
        brazo.set_named_target('go')
        if self.tries == 1:
            talk('Looking for object')
            head.set_joint_values([0.0, -0.77])

                
        rospy.sleep(0.8)
        res=segmentation_server.call(request)

        succ=seg_res_tf(res)

        print ('succ',succ)
        if succ:
        
            return 'succ'
        return'failed'


# --------------------------------------------------
class Move_arm_pregrasp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries', 'plan'])
        self.tries = 0

    def execute(self, userdata):

        rospy.loginfo('STATE : FK to pregrasp from above floor')
        

        self.tries += 1
        print(f'Try {self.tries} of 4 attempts')
        if self.tries == 4:
            self.tries=0
            return 'tries'


        head.set_named_target('neutral')
        rospy.sleep(0.5) #Wait for head before clearing octomap
        clear_octo_client()
        pickup_pose=[0.35,-1.2,0.0,-1.9, 0.0, 0.0]
        succ= arm.go(pickup_pose)
        
    
        if succ:
            return 'succ'
        else:
            return 'failed'
#########################################################################################################

# --------------------------------------------------
class Move_arm_grasp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):
        succ = False
        
        gripper.open()
        _,rot = tf_man.getTF(target_frame='base_link', ref_frame='map')
        original_rot= tf.transformations.euler_from_quaternion(rot)[2]
        target_object= 'object_0'
        while not succ:
            #trans,rot = tf_man.getTF(target_frame='static003_cracker_box', ref_frame='hand_palm_link')
            trans, _ = tf_man.getTF(target_frame=target_object, ref_frame='hand_palm_link')

            if type(trans) is not bool:
                eX, eY, eZ = trans
                eY+=-0.05
                eT= tf.transformations.euler_from_quaternion(rot)[2] - original_rot    #(known loc pickup change to line finder?)   NO! THERE ARE  ROUND TABLES !
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
            # if type(trans) is bool:
            else:
                trans,_ = tf_man.getTF(target_frame=target_object, ref_frame='hand_palm_link')
                _,rot = tf_man.getTF(target_frame='base_link', ref_frame='map')
                print('no tf')
                return 'failed'
        
        position, rotation = tf_man.getTF(target_frame=target_object, ref_frame='base_link')
        add_collision_object(position=position, rotation=rotation, object_name=target_object)
        clear_octo_client()
        av=arm.get_current_joint_values()
        print (av,'av')
        av[0]+= -0.17
        succ=arm.go(av)
        rospy.sleep(0.5)
        gripper.close()
        arm.set_named_target('go')
        arm.go()
        return 0

        
#########################################################################################################


class Get_close_to_object(smach.State):  # ADD KNONW LOCATION DOOR
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):

        rospy.loginfo('STATE : Navigate to known location')

        print(f'Try {self.tries} of 3 attempts')
        self.tries += 1
        if self.tries == 3:
            return 'tries'
        if self.tries == 1: talk('Getting Close')
        print('getting close to Target')
        head.to_tf('Target')
        res = omni_base.move_d_to(0.66,'Target')
        head.set_named_target('neutral')

        print(res)
        
        return 'succ'
        if res == 3:
            return 'succ'
        else:
            talk('Navigation Failed, retrying')
            return 'failed'




        
        
#########################################################################################################
        
# --------------------------------------------------
class Plan_arm(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):

        rospy.loginfo('STATE : plan arm using IK')
        

        self.tries += 1
        print(f'Try {self.tries} of 10 attempts')
        if self.tries == 10:
            self.tries=0
            return 'tries'  
        #t=tfBuffer.lookup_transform('hand_palm_link', 'Target',rospy.Time())
        clear_octo_client()
        pose, quat=tf_man.getTF('Target')

        wb_gp=whole_body.get_current_pose()
        


        wb_gp.pose.position.x= pose[0]
        wb_gp.pose.position.y= pose[1]
        wb_gp.pose.position.z= pose[2]+0.35

        whole_body.set_pose_target(wb_gp)
        plan=whole_body.plan()
        
        if plan[0]:
            self.tries=0
            return 'succ'
        else:
            return 'failed'
#########################################################################################################
        
                
# --------------------------------------------------
class Execute_plan(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):

        rospy.loginfo('STATE : EXECUTE arm using IK')
        

        self.tries += 1
        print(f'Try {self.tries} of 10 attempts')
        if self.tries == 10:
            self.tries=0
            return 'tries'  
        #t=tfBuffer.lookup_transform('hand_palm_link', 'Target',rospy.Time())
        
        succ=whole_body.go()

        pose, quat=tf_man.getTF('Target',   )
        pos,rot=tf_man.getTF(target_frame='hand_palm_link',ref_frame='Target')
        print (pos,rot,'pos,rot')

        if succ:
            return 'succ'
        else:
            clear_octo_client()
            return 'failed'
#########################################################################################################
        
        
        

# --------------------------------------------------


class Goto_exit(smach.State):  # ADD KNONW LOCATION DOOR
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):

        rospy.loginfo('STATE : Navigate to known location')

        print(f'Try {self.tries} of 3 attempts')
        self.tries += 1
        if self.tries == 3:
            return 'tries'
        if self.tries == 1: talk('Navigating to, exit')
        #print ('navigating to room'+str(self.next_room))
        res = omni_base.move_base(known_location='exit')
        print(res)

        if res == 3:
            talk('test done! thanks for your attenntion!')
            return 'succ'

        else:
            talk('Navigation Failed, retrying')
            return 'failed'


# --------------------------------------------------
def init(node_name):
    print('smach ready')

# --------------------------------------------------
# Entry point
if __name__ == '__main__':
    global whole_body
    print("Takeshi STATE MACHINE...")
    init("takeshi_smach")
    # State machine, final state "END"
    sm = smach.StateMachine(outcomes=['SUCCESS','PREEMPTED','FAILED'])
    whole_body=moveit_commander.MoveGroupCommander('whole_body')
    # sm.userdata.clear = False
    sis = smach_ros.IntrospectionServer('SMACH_VIEW_SERVER', sm, '/SM_PICKUP_FLOOR')
    sis.start()

    with sm:
        # State machine for Restaurant
        smach.StateMachine.add("INITIAL",       Initial(),      transitions={'failed': 'INITIAL',       'succ': 'FIND_OBJECT',       'tries': 'INITIAL'})
        #################################################################################################################################################################
        #smach.StateMachine.add("WAIT_DOOR",     Wait_door(),    transitions={'failed': 'WAIT_DOOR',     'succ': 'ENTER_ARENA',      'tries': 'WAIT_DOOR'})
        smach.StateMachine.add("FIND_OBJECT",   Find_object(),   transitions={'failed': 'FIND_OBJECT',       'succ': 'MOVE_ARM_PREGRASP',       'tries': 'FAILED'})
        smach.StateMachine.add("GET_CLOSE_TO_OBJECT",  Get_close_to_object(),   transitions={'failed': 'GET_CLOSE_TO_OBJECT',       'succ': 'MOVE_ARM_PREGRASP',       'tries': 'INITIAL'})
        
        smach.StateMachine.add("MOVE_ARM_PREGRASP",  Move_arm_pregrasp(),   transitions={      'failed': 'MOVE_ARM_PREGRASP', 'succ': 'MOVE_ARM_GRASP',      'tries': 'INITIAL','plan':'EXECUTE_PLAN'})
        smach.StateMachine.add("MOVE_ARM_GRASP",  Move_arm_grasp(),   transitions={      'failed': 'FAILED', 'succ': 'SUCCESS',      'tries': 'INITIAL'})

        smach.StateMachine.add("PLAN_ARM",           Plan_arm(),      transitions={      'failed': 'PLAN_ARM', 'succ': 'EXECUTE_PLAN',      'tries': 'FIND_OBJECT'})
        smach.StateMachine.add("EXECUTE_PLAN",      Execute_plan(),   transitions={      'failed': 'EXECUTE_PLAN', 'succ': 'SUCCESS',      'tries': 'FAILED'})
        smach.StateMachine.add("WAIT_HAND",     Wait_push_hand(),       transitions={'failed': 'WAIT_HAND',    'succ': 'GOTO_EXIT', 'tries': 'WAIT_HAND'})
        smach.StateMachine.add("GOTO_EXIT",     Goto_exit(),     transitions={'failed': 'SUCCESS','succ': 'SUCCESS'    , 'tries': 'SUCCESS'})
        ##################################################################################################################################################################
        


    asw = ActionServerWrapper(
        'grasp_floor_act_server', PickUpAction,
        wrapped_container = sm,
        succeeded_outcomes = ['SUCCESS'],
        aborted_outcomes = ['FAILED'],
        preempted_outcomes = ['PREEMPTED'] )
    asw.run_server()

    outcome = sm.execute()


#!/usr/bin/env python3

from smach_utils2 import *
from smach_ros import ActionServerWrapper
from action_pickup_floor.msg import PickUpAction
##### Define state INITIAL #####

# --------------------------------------------------
class Initial(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):

        rospy.loginfo('STATE : INITIAL')
        print('Robot neutral pose')
        self.tries += 1
        print(f'Try {self.tries} of 5 attempts')
        if self.tries == 3:
            return 'tries'
        clean_knowledge()
        head.set_named_target('neutral')
        #print('head listo')
        #brazo.set_named_target('go')
        #print('brazo listo')
        rospy.sleep(0.8)

        return 'succ'

#-------------------------------------------------

class Wait_door(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'])
        self.tries=0
    def execute(self,userdata):
        rospy.loginfo('STATE : Wait_door')
        print('robot neutral pose')
        print('Try',self.tries,'of 100 attempts') 
        self.tries+=1
        rospy.sleep(1.0)
        if self.tries == 1:
            talk('I am waiting for door to be opened')
            rospy.sleep(0.7)

        if not line_detector.line_found():
            rospy.sleep(0.7)
            talk('I can see the door is opened, entering')
            rospy.sleep(0.7)
            return 'succ'
        else:
            return 'tries'

# --------------------------------------------------


class Enter_arena(smach.State):  # ADD KNONW LOCATION DOOR
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):

        rospy.loginfo('STATE : Navigate to known location')

        print(f'Try {self.tries} of 3 attempts')
        self.tries += 1
        if self.tries == 3:
            return 'tries'
        if self.tries == 1: talk('I am entering to, arena')
        #res = omni_base.move_base(known_location='door')
        #print(res)
        rospy.sleep(8.0)
        return 'succ'
        if res == 3:
            return 'succ'
        else:
            talk('Navigation Failed, retrying')
            return 'failed'

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




        #if self.tries == 1: talk('Looking for object')
        brazo.set_named_target('go')
        if self.tries==1:head.set_joint_values([0.0, -0.77])
        
        rospy.sleep(0.8)
        res=segmentation_server.call()
        if len(res.poses.data)==0: return 'failed'
        else:
            print('object found')
            poses=np.asarray(res.poses.data)
            poses=poses.reshape((int(len(poses)/3) ,3     )      )  





            tf_man.pub_static_tf(pos=poses[0,:], point_name='target', ref='head_rgbd_sensor_rgb_frame')## which object to choose   #TODO
            rospy.sleep(0.3)
            tf_man.change_ref_frame_tf(point_name='target', new_frame='map')
            rospy.sleep(0.3)
            pose, _ =tf_man.getTF('target')
            tf_man.pub_static_tf(pos=pose, point_name='Target')## which object to choose   #TODO


            head.set_named_target('neutral')
            self.tries=0
            return 'succ'


# --------------------------------------------------
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


        clear_octo_client()
    
        arm_grasp_from_above = [0.2, -1.57, -0.13 , -1.57, 0.0, 0.0]
        succ = arm.go(arm_grasp_from_above)   
    
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

        rospy.loginfo('STATE : Proportional... (POTFIELDS) to pregrasp from above floor')
        

        self.tries += 1
        if self.tries == 1:
            head.set_named_target('neutral')
            head.turn_base_gaze('target')
            head.set_named_target('down')
        #pre grasp pose
        brazo.set_named_target('grasp_floor')
        gripper.open()
        rospy.sleep(0.8)
        #get close to object
        brazo.move_hand_to_target(target_frame='target', THRESHOLD=0.01 )
        #move down hand 
        acp = brazo.get_joint_values()
        acp[0]-= 0.04
        brazo.set_joint_values(acp)
        rospy.sleep(0.5)
        #grasp
        gripper.close()
        #move up to check if grasped
        #acp = brazo.get_joint_values()
        acp[0] += 0.09
        brazo.set_joint_values(acp)
        if brazo.check_grasp(weight = 1.0):
            brazo.set_named_target('neutral')
            talk('I took the object')
            rospy.sleep(0.8)
            return 'succ'
        else:
            talk('I could not take the object, i will try again')
            return 'failed'

        
#########################################################################################################
        
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
        smach.StateMachine.add("FIND_OBJECT",  Find_object(),   transitions={'failed': 'FIND_OBJECT',       'succ': 'GET_CLOSE_TO_OBJECT',       'tries': 'FAILED'})
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


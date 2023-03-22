#!/usr/bin/env python3

from smach_utils2 import *


##### Define state INITIAL #####

class Initial(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):

        rospy.loginfo('STATE : INITIAL')
        print('robot neutral pose')

        print('Try', self.tries, 'of 5 attempts')
        self.tries += 1
        if self.tries == 3:
            return 'tries'

        # clear_octo_client()

        # scene.remove_world_object()
        # Takeshi neutral
        # head.set_named_target('neutral')
        # succ=head.go()
        head.set_joint_values([0.0, 0.0])
        rospy.sleep(0.8)

        # arm.set_named_target('go')
        # succ=arm.go()

        succ = True

        if succ:
            return 'succ'
        else:
            return 'failed'


class Wait_push_hand(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):

        rospy.loginfo('STATE : Wait for Wait_push_hand')
        print('Waiting for hand to be pushed')

        print('Try', self.tries, 'of 3 attempts')
        self.tries += 1
        if self.tries == 3:
            return 'tries'
        talk('Gently, ...  push my hand to begin')
        succ = wait_for_push_hand(100)

        if succ:
            return 'succ'
        else:
            return 'failed'


class Scan_face(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=['succ', 'unknown', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):

        rospy.loginfo('State : SCAN_FACE')
        self.tries += 1

        if self.tries == 1:
            # head.set_named_target('neutral')
            # head.go()
            head.set_joint_values([0.0, 0.0])
        elif self.tries == 2:
            # hv= head.get_current_joint_values()
            # hv[0]= -0.6
            # hv[1]= 0.0
            # head.go(hv)
            head.set_joint_values([-0.6, 0.0])
        elif self.tries == 3:
            # hv= head.get_current_joint_values()
            # hv[0]= 0.6
            # hv[1]= 0.0
            # head.go(hv)
            head.set_joint_values([0.6, 0.0])
        elif self.tries >= 4:
            self.tries = 0
            return'tries'
        rospy.sleep(0.8)

        res = wait_for_face(3)  # default 10 secs

        print('Checking for faces')
        if res == None:
            return 'failed'
        if res != None:
            print('RESPONSE', res.Ids.ids[0])
            if res.Ids.ids[0].data == 'NO_FACE':
                print('No face Found Keep scanning')
                return 'failed'
            else:
                print('A face was found.')
                if (res.Ids.ids[0].data == 'unknown'):
                    talk('I believe I do not know you')

                    return 'unknown'

                name=res.Ids.ids[0].data
                talk('I found you, I Think you are .' + name)
                print(res.Angs.data)

                
                points = rgbd.get_points()
                trans = bbox_3d_mean(points, np.asarray(res.Angs.data))
                print(trans)
                #############################################################################################
                ##############################################################################################
                tf_man.pub_static_tf(
                    pos=trans, point_name=name, ref='head_rgbd_sensor_link')
                #tf_man.pub_static_tf(pos=trans, point_name=res.Ids.ids[0].data, ref='head_rgbd_sensor_link')
                rospy.sleep(0.3)
                tf_man.change_ref_frame_tf(res.Ids.ids[0].data)
                #################################################################
                #res=omni_base.move_base(goal_x= goal_D[0] , goal_y = goal_D[1], goal_yaw= tf.transformations.euler_from_quaternion(robotquat)[2]    )
                #res=move_base(goal_x= goal_D[0] , goal_y = goal_D[1], goal_yaw= ((tf.transformations.euler_from_quaternion(robotquat)[2])+np.arctan2(delta[1], delta[0]))%2*np.pi  )
                #################################################################################################################################
                #print (hcp)
                # head.set_joint_value_target(hcp)
                print(res)
                try:
                    trans,quat = tf_man.getTF(target_frame=name)
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    print ( 'No TF FOUND')
                omni_base.move_d_to(1.5, name )
                print (trans)
                #head.absolute(*trans)

                talk (name +'... I will lead you to the living room, please follow me')
                
                return 'succ'


class New_face(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):

        rospy.loginfo('STATE : NEW_FACE')

        self.tries += 1
        if self.tries == 3:
            return 'tries'

        # head.set_named_target('neutral')
        head.set_joint_values([0.0, 0.0])
        rospy.sleep(0.8)
        # succ=head.go()
        talk('Please, tell me your name')
        res = speech_recog_server()
        talk('Hello' + res.data)
        img = rgbd.get_image()
        res2 = train_face(img, res.data)
        print(res2)
        talk('I am  learning your face, please stare at me')

        # arm.set_named_target('go')
        # succ=arm.go()

        succ = True
        if succ:
            return 'succ'
        else:
            return 'failed'


class Goto_living_room(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):

        rospy.loginfo('STATE : navigate to known location')

        print('Try', self.tries, 'of 3 attempts')
        self.tries += 1
        if self.tries == 3:
            return 'tries'
        talk('Navigating to ,living room')
        res = omni_base.move_base(known_location='living_room')
        print(res)

        if res == 3:
            return 'succ'
        else:
            talk('Navigation Failed, retrying')
            return 'failed'


class Goto_living_room_2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):

        rospy.loginfo('STATE : navigate to known location_2')

        print('Try', self.tries, 'of 3 attempts')
        self.tries += 1
        if self.tries == 3:
            return 'tries'
        talk('Navigating to ,living room 2')
        res = omni_base.move_base(known_location='living_room')
        robot, robotquat = tf_man.getTF('base_link')
        new_yaw = (tf.transformations.euler_from_quaternion(
            robotquat)[2]+np.pi) % 2*np.pi

        # go to living room an do a 180
        res = omni_base.move_base(robot[0], robot[1], new_yaw)
        print(res)

        if res == 3:
            return 'succ'
        else:
            talk('Navigation Failed, retrying')
            return 'failed'


class Goto_face(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):

        rospy.loginfo('STATE : navigate to known location')

        print('Try', self.tries, 'of 3 attempts')
        self.tries += 1
        if self.tries == 3:
            return 'tries'
        #res= omni_base.move_D

        if res == 3:
            return 'succ'
        else:
            talk('Navigation Failed, retrying')
            return 'failed'


class Find_sitting_place(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):

        rospy.loginfo('STATE : Looking for a place to sit')

        print('Try', self.tries, 'of 3 attempts')
        self.tries += 1

        # gaze.to_tf(target_frame= 'sofa')### RUBEN GFAZE TO TF
        hcp = head.absolute(10, -2.0, 1.0)

        if self.tries == 2:
            # hv= head.get_current_joint_values()
            # hv[0]= -0.6
            # hv[1]= 0.0
            # head.go(hv)
            head.set_joint_values([-0.6, 0.0])
        elif self.tries == 3:
            # hv= head.get_current_joint_values()
            # hv[0]= 0.6
            # hv[1]= 0.0
            # head.go(hv)
            head.set_joint_values([0.6, 0.0])
        elif self.tries >= 4:
            self.tries = 0
            return'tries'
        rospy.sleep(0.8)

        res = wait_for_face(3)  # seconds

        if res == None:

            talk('Here is a place to sit.')
            arm.set_named_target('neutral')
            arm.go()
            return 'succ'

        if res != None:
            return 'failed'


def init(node_name):
    print('smach ready')


# Entry point
if __name__ == '__main__':
    print("Takeshi STATE MACHINE...")
    init("takeshi_smach")
    # State machine, final state "END"
    sm = smach.StateMachine(outcomes=['END'])

    #sm.userdata.clear = False
    sis = smach_ros.IntrospectionServer(
        'SMACH_VIEW_SERVER', sm, '/SM_RECEPTIONIST')
    sis.start()

    with sm:
        # State machine for Restaurant

        smach.StateMachine.add("INITIAL",Initial(),transitions={'failed': 'INITIAL',          'succ': 'WAIT_PUSH_HAND',           'tries': 'END'})
        smach.StateMachine.add("SCAN_FACE",   Scan_face(),  transitions={'failed': 'SCAN_FACE',  'unknown': 'NEW_FACE',       'succ': 'GOTO_LIVING_ROOM','tries': 'INITIAL'})
        smach.StateMachine.add("WAIT_PUSH_HAND",   Wait_push_hand(),  transitions={'failed': 'WAIT_PUSH_HAND',  'succ': 'SCAN_FACE',    'tries': 'END'})
        smach.StateMachine.add("NEW_FACE",           New_face(),      transitions={'failed': 'INITIAL',          'succ': 'END',           'tries': 'END'})
        smach.StateMachine.add("GOTO_FACE",           Goto_face(),    transitions={'failed': 'GOTO_FACE',          'succ': 'GOTO_LIVING_ROOM',           'tries': 'END'})
        smach.StateMachine.add("GOTO_LIVING_ROOM",Goto_living_room(),transitions={'failed': 'GOTO_LIVING_ROOM',          'succ': 'FIND_SITTING_PLACE','tries': 'END'})
        smach.StateMachine.add("FIND_SITTING_PLACE", Find_sitting_place(), transitions={'failed': 'GOTO_LIVING_ROOM',          'succ': 'END',           'tries': 'END'})
        smach.StateMachine.add("GOTO_LIVING_ROOM_2", Goto_living_room_2(),transitions={'failed': 'GOTO_LIVING_ROOM',          'succ': 'END',           'tries': 'END'})
        #smach.StateMachine.add("SIT_GUEST",           Sit_guest(),          transitions = {'failed':'GOTO_LIVING_ROOM',          'succ':'INTRODUCE_GUEST'      ,           'tries':'END'})
        #smach.StateMachine.add("INTRODUCE_GUEST",           Introduce_guest(),          transitions = {'failed':'GOTO_LIVING_ROOM',          'succ':'INTRODUCE_GUEST'      ,           'tries':'END'})

    outcome = sm.execute()

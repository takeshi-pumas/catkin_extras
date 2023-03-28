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

class Goto_door(smach.State):   ###ADD KNONW LOCATION DOOR
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):

        rospy.loginfo('STATE : navigate to known location')

        print('Try', self.tries, 'of 3 attempts')
        self.tries += 1
        if self.tries == 3:
            return 'tries'
        if self.tries==1:talk('Navigating to door')
        res = omni_base.move_base(known_location='door')
        print(res)

        if res == 3:
            self.tries=0
            return 'succ'
        else:
            talk('Navigation Failed, retrying')
            return 'failed'


class Scan_face(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=['succ', 'unknown', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):
        global img_face,name_face
        talk('Scanning for faces')

        rospy.loginfo('State : SCAN_FACE')
        self.tries += 1
        head.set_joint_values([0.0, 0.3])
        """if self.tries == 1:
                                    # head.set_named_target('neutral')
                                    # head.go()
                                    head.set_joint_values([0.0, 0.3])
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
                                    elif self.tries >= 4:"""
        if self.tries >= 4:
            self.tries = 0
            return'tries'
        
        rospy.sleep(0.5)
        res,img_face = wait_for_face()  # default 10 secs

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

                


                #points = rgbd.get_points()                              ##FACE POS FROM FACE RECOG
                #boundRect=np.asarray(res.Angs.data).astype('int')       ##FACE POS FROM FACE RECOG
                #trans = bbox_3d_mean(points, boundRect)                 ##FACE POS FROM FACE RECOG
                

                trans_dict=human_detect_server.call()                   ##FROM HUMAN FINDER OPEN POSE
                trans =[trans_dict.x,trans_dict.y,trans_dict.z]         ##FROM HUMAN FINDER OPEN POSE
                print( trans )                                          ##FROM HUMAN FINDER OPEN POSE
                
                #############################################################################################
                ##############################################################################################
                tf_man.pub_static_tf(pos=trans, point_name=name, ref='head_rgbd_sensor_link')
                
                rospy.sleep(0.3)
                tf_man.change_ref_frame_tf(name)
                
                
                try:
                    trans,quat = tf_man.getTF(target_frame=name)
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    print ( 'No TF FOUND')
                omni_base.move_d_to(1, name )
                head.to_tf(name)

                print (trans)
                #head.absolute(*trans)

                talk (name +'... I will lead you to the living room, please follow me')
                name_face=name
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
            robotquat)[2]+np.pi)

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

        res , _ = wait_for_face(5)  # seconds
        print (res)
        if res == None:

            talk('Here is a place to sit.')
            #arm.set_named_target('neutral')
            #arm.go()
            bcp = brazo.get_joint_values()
            bcp[2] = 0.0
            brazo.set_joint_values(bcp)
            return 'succ'

        if res != None:
            return 'failed'
################################################################
         
class Introduce_guest(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):

        rospy.loginfo('STATE : Introduce_guest')####Using ANALYZE FACE SERVICE (DEEP FACE SERVER  on FACE_RECOG  PKG)
                                                ############### #analyze = rospy.ServiceProxy('analyze_face', RecognizeFace)    


        print('Try', self.tries, 'of 3 attempts')
        self.tries += 1
        if self.tries == 3:
            return 'tries'
        


        takeshi_line= analyze_face_from_image(img_face,name_face)   
        
        print (takeshi_line)
        if (len(takeshi_line)!=0):
            talk(takeshi_line)
            rospy.sleep(2)
            return 'succ'
        else :
            print ('no face in img deep analyze')
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
        smach.StateMachine.add("WAIT_PUSH_HAND",   Wait_push_hand(),  transitions={'failed': 'WAIT_PUSH_HAND',  'succ': 'GOTO_DOOR',    'tries': 'END'})
        smach.StateMachine.add("SCAN_FACE",   Scan_face(),  transitions={'failed': 'SCAN_FACE',  'unknown': 'NEW_FACE',       'succ': 'GOTO_LIVING_ROOM','tries': 'GOTO_DOOR'})
        smach.StateMachine.add("GOTO_DOOR",Goto_door(),transitions={'failed': 'GOTO_DOOR',          'succ': 'SCAN_FACE','tries': 'SCAN_FACE'})
        smach.StateMachine.add("NEW_FACE",           New_face(),      transitions={'failed': 'INITIAL',          'succ': 'END',           'tries': 'SCAN_FACE'})
        smach.StateMachine.add("GOTO_FACE",           Goto_face(),    transitions={'failed': 'GOTO_FACE',          'succ': 'GOTO_LIVING_ROOM',           'tries': 'SCAN_FACE'})
        smach.StateMachine.add("GOTO_LIVING_ROOM",Goto_living_room(),transitions={'failed': 'GOTO_LIVING_ROOM',          'succ': 'FIND_SITTING_PLACE','tries': 'END'})
        smach.StateMachine.add("FIND_SITTING_PLACE", Find_sitting_place(), transitions={'failed': 'GOTO_LIVING_ROOM',          'succ': 'INTRODUCE_GUEST',           'tries': 'GOTO_DOOR'})
        smach.StateMachine.add("INTRODUCE_GUEST",           Introduce_guest(),          transitions = {'failed':'GOTO_LIVING_ROOM',          'succ':'INTRODUCE_GUEST'      ,           'tries':'END'})
        smach.StateMachine.add("GOTO_LIVING_ROOM_2", Goto_living_room_2(),transitions={'failed': 'GOTO_LIVING_ROOM',          'succ': 'FIND_SITTING_PLACE',           'tries': 'GOTO_DOOR'})
        #smach.StateMachine.add("SIT_GUEST",           Sit_guest(),          transitions = {'failed':'GOTO_LIVING_ROOM',          'succ':'INTRODUCE_GUEST'      ,           'tries':'END'})

    outcome = sm.execute()

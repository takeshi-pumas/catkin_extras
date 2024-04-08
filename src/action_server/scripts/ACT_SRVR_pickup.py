#!/usr/bin/env python3

import rospy
import smach
from smach_ros import ActionServerWrapper, IntrospectionServer, SimpleActionState
from object_classification.srv import *
from action_server.msg import GraspAction, PickUpAction
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import Empty
from cv_bridge import CvBridge
from utils import misc_utils, grasp_utils, nav_utils


rospy.init_node('pickup_node')
bridge = CvBridge()
classify_client = rospy.ServiceProxy('/classify', Classify)
clear_octo_client = rospy.ServiceProxy('/clear_octomap', Empty)

rgbd = misc_utils.RGBD()
head = grasp_utils.GAZE()
omni_base = nav_utils.NAVIGATION()
voice = misc_utils.TALKER()
tf_man = misc_utils.TF_MANAGER()


class ServeBreakfastSMACH:
    def __init__(self):
        
        rospy.loginfo("Initializing Serve Breakfast SMACH")


        self.sm = smach.StateMachine(outcomes=['END'], input_keys=["goal"])
        with self.sm:
            smach.StateMachine.add("INITIAL", Initial(),
                                   transitions={'failed': 'INITIAL', 'succ': 'GOTO_KITCHEN_CONTAINER'})
            smach.StateMachine.add("GOTO_KITCHEN_CONTAINER", GotoTable(),
                                   transitions={'failed': 'GOTO_KITCHEN_CONTAINER', 'succ': 'SCAN_CONTAINER'})
            smach.StateMachine.add("SCAN_CONTAINER", ScanTable(),
                                   transitions={'failed': 'SCAN_CONTAINER', 'succ': 'DECIDE_GRASP'})

            smach.StateMachine.add("DECIDE_GRASP", DecideGrasp(),
                                   transitions={'failed': 'SCAN_CONTAINER', 'succ': 'GRASP_GOAL'})
            smach.StateMachine.add("GRASP_GOAL", SimpleActionState('/grasp_server', GraspAction, goal_slots=['target_pose']),
                                   transitions={'preempted': 'END', 'succeeded': 'END', 'aborted': 'END'})

        self.sis = IntrospectionServer('serve_breakfast_smach_introspection', self.sm, '/SM_ROOT')
        self.sis.start()

        self.wrapper = self.wrapper = ActionServerWrapper("pickup_server", PickUpAction,
                                           wrapped_container = self.sm,
                                           succeeded_outcomes=["succeeded"],
                                           aborted_outcomes=["failed"],
                                           preempted_outcomes=["preempted"],
                                           goal_key='goal',
                                           result_key="action_done")
        self.wrapper.run_server()

    def execute_cb(self, goal):
        rospy.loginfo('Received action goal: %s', goal)
        self.sm.userdata.goal = goal
        outcome = self.sm.execute()
        self.wrapper.server.set_succeeded()
        outcome = self.sm.execute()

class Initial(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed'])
        self.tries = 0

    def execute(self, userdata):
        self.tries += 1
        rospy.loginfo('STATE: INITIAL')
        rospy.loginfo(f'Try {self.tries} of 5 attempts')
        obj = "object"
        # voice.talk(f"Ready to pick up {obj} from table ")  # Uncomment this if you have implemented it
        return 'succ'

class GotoTable(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed'])
        self.tries = 0

    def execute(self, userdata):
        rospy.loginfo("SMACH: Go to breakfast container")
        self.tries += 1
        res = omni_base.move_base(known_location='kitchen_container')
        print(res)

        if res:
            self.tries = 0
            return 'succ'
        else:
            # voice.talk('Navigation failed, retrying')  # Uncomment this if you have implemented it
            return 'failed'

class ScanTable(smach.State):
    def __init__(self):
        smach.State.__init__(self, output_keys=['obj_detected'], outcomes=['succ', 'failed'])
        self.tries = 0

    def execute(self, userdata):
        rospy.loginfo("SMACH: Scan breakfast container")
        self.tries += 1
        # head_values = [0.0,-0.9]
        head_values = [0.0, -0.5]
        head.set_joint_values(head_values)
        rospy.sleep(1.0)
        # image= cv2.cvtColor(rgbd.get_image(), cv2.COLOR_RGB2BGR)
        img_msg = bridge.cv2_to_imgmsg(rgbd.get_image())
        req = classify_client.request_class()
        req.in_.image_msgs.append(img_msg)
        res = classify_client(req)

        userdata.obj_detected = [res.names, res.poses]  # , res.confidence

        return 'succ'

class DecideGrasp(smach.State):
    def __init__(self):
        smach.State.__init__(self, input_keys=['obj_detected'], output_keys=['target_pose'], outcomes=['succ', 'failed'])
        self.tries = 0
        self.order_to_grasp = ["bowl", "cereal", "milk", "spoon"]

    def execute(self, userdata):
        rospy.loginfo("SMACH: Decide which object is better to grasp")
        self.tries += 1

        succ = False
        position = []
        for idx, obj_name in enumerate(userdata.obj_detected[0]):
            obj = obj_name.data.split('_', 1)
            print(obj)
            target_object = 'dice'
            if obj[1] == target_object:
                succ = True
                position.append(userdata.obj_detected[1][idx].position.x)
                position.append(userdata.obj_detected[1][idx].position.y)
                position.append(userdata.obj_detected[1][idx].position.z)
                tf_man.pub_static_tf(pos=position, point_name=target_object, ref='head_rgbd_sensor_rgb_frame')
                rospy.sleep(0.8)
                break
        if succ:
            clear_octo_client()
            pos, _ = tf_man.getTF(target_frame=target_object, ref_frame='odom')
            target_pose = Float32MultiArray()
            # pos[1] += 0.03 #only bowl grasp
            pos[2] += 0.04  # 0.03

            target_pose.data = pos
            userdata.target_pose = target_pose
            return 'succ'
        else:
            return 'failed'


if __name__ == '__main__':
    try:
        serve_breakfast_smach = ServeBreakfastSMACH()
        #serve_breakfast_smach.run()
    except rospy.ROSInterruptException:
        pass

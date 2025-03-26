#! /usr/bin/env python3

import tf
import tf2_ros
import numpy as np
import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from hmm_navigation.msg import NavigateAction, NavigateActionGoal, NavigateActionFeedback, NavigateActionResult
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PointStamped, PoseStamped, Quaternion, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Empty, String

from utils.grasp_utils import *
import rospkg
import yaml


##################################################
# Por probar
class nav_status():
    def __init__(self):
        self.suscriber = rospy.Subscriber(
            '/navigation/status',
            GoalStatus, self._callback)
        self._status = None

    def _callback(self, goal_status):
        self._status = goal_status.status

    def get_status(self):
        return self._status

##################################################
# YAML reader

## PARAM  FILE
def read_yaml(known_locations_file='/known_locations.yaml'):
    rospack = rospkg.RosPack()
    file_path = rospack.get_path('config_files') + known_locations_file

    with open(file_path, 'r') as file:
        content = yaml.safe_load(file)
    return content


def match_location(location):
    content = read_yaml()
    try:
        return True, content[location]
    except:
        return False, 'No location found'
##################################################
# Navigation functions


def pose2feedback(pose_robot, quat_robot, timeleft, euclD , anglD):
    feed = NavigateActionFeedback()
    feed.feedback.x_robot = pose_robot[0]
    feed.feedback.y_robot = pose_robot[1]
    _, _, yaw = tf.transformations.euler_from_quaternion(quat_robot)
    feed.feedback.yaw_robot = yaw
    feed.feedback.timeleft = timeleft
    feed.feedback.euclD = euclD
    feed.feedback.anglD = anglD
    return feed


def goal_police(goal):
    # goal corrector
    goal_corrected = goal

    return goal_corrected

##################################################


class pumas_navServer():
    def __init__(self):
        self.pumas_nav_server = actionlib.SimpleActionServer(
            "navigate", NavigateAction,
            execute_cb=self.execute_cb,
            auto_start=False)
        self.pumas_nav_server.start()

    def execute_cb(self, goal):
        #########
        # goal police
        #goal_corrected = goal_police(goal)
        #########
        success = False

        # Matching known location if given
        x, y, yaw = goal.x, goal.y, goal.yaw
        known_loc = goal.known_location.casefold()
        if known_loc != 'None':
            succ, loc = match_location(known_loc)
            if succ:
                XYT = loc[:3]
                # it could change
                x, y, yaw = XYT[0]['x'], XYT[1]['y'], XYT[2]['theta']
                goal.x, goal.y, goal.yaw = x, y, yaw

        head.set_joint_values(head_pose=[0.0, -0.5])
        # Fill result message (could be modified)
        result = NavigateActionResult()
        rate = rospy.Rate(10)
        timeout = rospy.Time.now().to_sec() + goal.timeout
        rot = tf.transformations.quaternion_from_euler(0, 0, yaw)
        goal_pose = PoseStamped()
        goal_pose.pose.position = Point(x, y, 0)
        goal_pose.pose.orientation = Quaternion(*rot)
        goal_nav_publish.publish(goal_pose)

        # Feedback publisher and timeout checker
        while (timeout >= rospy.Time.now().to_sec()) and not success:
            # Goal distance and orientation calculation
            c_pose, q_rob = tf_man.getTF(target_frame='base_link')
            _, _, rob_yaw = tf.transformations.euler_from_quaternion(q_rob)
            #rob_yaw = rob_yaw % (2 * np.pi)
            #anglD = (yaw - rob_yaw)
            anglD = (yaw - rob_yaw)

                #UGLY
            if anglD < -2*np.pi :
                anglD= -1*anglD%(2*np.pi)
            elif anglD > 2*np.pi :
                anglD= anglD%(2*np.pi)


            if anglD < -np.pi :
                anglD= (anglD%np.pi)
            elif anglD > np.pi:
                anglD=(anglD%np.pi)*-1

            euclD = np.linalg.norm(np.asarray((x, y)) - c_pose[:2])
            #print (anglD)
            timeleft = timeout - rospy.Time.now().to_sec()
            if timeleft > 0:
                #print(timeleft, 'timeleft')
                feed = pose2feedback(c_pose, q_rob, timeleft, euclD,anglD)
                self.pumas_nav_server.publish_feedback(feed.feedback)

            

            close_enough = euclD < 0.45
            if close_enough: 
                speed=Twist()
                angle_enough=abs(anglD) < 0.1
                if angle_enough: 
                    speed.angular.z=0
                    pub.publish(speed)
                    pub_stop.publish()
                    success =  True

                else:
                    if anglD < 0 : 
                        speed.angular.z = -0.35
                    if anglD >= 0 : 
                        speed.angular.z = +0.35
                    pub.publish(speed)
                    rospy.sleep(.3)


        # state = NS.get_status()
        head.set_joint_values(head_pose = [0.0, 0.0])
        rospy.sleep(0.8)
        pub_stop.publish()
        if not success:
            self.pumas_nav_server.set_aborted()
        else:
            self.pumas_nav_server.set_succeeded()


if __name__ == "__main__":
    global goal_nav_publish, pub_stop, tf_man
    rospy.init_node('pumas_navigation_actionlib_server')

    pub = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=1)
      
    head = GAZE()
    goal_nav_publish = rospy.Publisher(
        '/move_base_simple/goal', PoseStamped, queue_size=1)
    tf_man = TF_MANAGER()
    pub_stop = rospy.Publisher('/navigation/stop', Empty, queue_size=10)
    print('pumas nav action server available')
    # NS = nav_status()
    s = pumas_navServer()
    rospy.spin()

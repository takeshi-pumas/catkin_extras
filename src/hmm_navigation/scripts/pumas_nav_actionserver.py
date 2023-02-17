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

import rospkg
import yaml


##################################################
#Por probar
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
#YAML reader
def read_yaml(known_locations_file = '/known_locations.yaml'):
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
#Navigation functions 
def pose2feedback(pose_robot,quat_robot,timeleft,euclD):
    feed = NavigateActionFeedback()
    feed.feedback.x_robot   = pose_robot[0]
    feed.feedback.y_robot   = pose_robot[1]
    # euler = tf.transformations.euler_from_quaternion((quat_robot[0] ,quat_robot[1] ,quat_robot[2] ,quat_robot[3])) 
    _,_,yaw = tf.transformations.euler_from_quaternion(quat_robot) 
    feed.feedback.yaw_robot = yaw
    feed.feedback.timeleft  = timeleft
    feed.feedback.euclD = euclD
    return feed


def goal_police(goal):
    #goal corrector
    goal_corrected = goal


    return goal_corrected

##################################################
class pumas_navServer():
    def __init__(self):
        self.pumas_nav_server = actionlib.SimpleActionServer(
            "navigate",NavigateAction, 
            execute_cb=self.execute_cb, 
            auto_start=False)
        self.pumas_nav_server.start()
    
    def execute_cb(self, goal):
        #########
        #goal police
        #goal_corrected = goal_police(goal)
        #########

        #Matching known location if given
        x,y,yaw = 0,0,0
        known_loc = goal.known_location.casefold()
        if known_loc != 'None':
            succ,loc = match_location(known_loc)
            if succ:
                XYT = loc[:3]
                x, y, yaw = XYT[0]['x'], XYT[1]['y'], XYT[2]['theta'] #it could change
                goal.x, goal.y, goal.yaw = x, y, yaw
        else:
            x, y, yaw = goal.x, goal.y, goal.yaw
        print (goal)

        #Fill result message (could be modified)
        result = NavigateActionResult()
        rate = rospy.Rate(10)
        timeout = rospy.Time.now().to_sec() + goal.timeout
        rot = tf.transformations.quaternion_from_euler(0, 0, yaw)
        goal_pose = PoseStamped()
        goal_pose.pose.position = Point(x,y,0)
        goal_pose.pose.orientation = Quaternion(*rot)
        goal_nav_publish.publish(goal_pose)

        # result.result.success = 2
        i = 0

        l_pose, _ = listener.lookupTransform('/map', 'base_link', rospy.Time(0))
        movement = False
        #Feedback publisher and timeout checker
        while timeout >= rospy.Time.now().to_sec():

            #Goal distance and orientation calculation
            l_pose = c_pose
            c_pose, q_rob = listener.lookupTransform('/map', 'base_link', rospy.Time(0))
            _,_,rob_yaw = tf.transformations.euler_from_quaternion(q_rob)
            rob_yaw = rob_yaw % (2 * np.pi)
            euclD = np.linalg.norm(np.asarray((x,y)) - c_pose[:2])
            anglD = yaw - rob_yaw

            timeleft = timeout - rospy.Time.now().to_sec()  
            print (timeleft,'timeleft')   
            feed = pose2feedback(c_pose, q_rob, timeleft, euclD)
            self.pumas_nav_server.publish_feedback(feed.feedback)

            movement = np.linalg.norm(np.asarray(c_pose) - np.asarray(l_pose)) >= 0.05 
            state = NS.get_status()

            if state == 1:
                if not movement:
                    pub_stop.publish()
                    goal_nav_publish.publish(goal_pose)
                else:
                    timeout = rospy.Time.now().to_sec() + 10
            elif state == 3:
                self.pumas_nav_server.set_succeeded()
            else:
                print('Else condition')

            #Por revisar
            # state = NS.get_status()
            # print(f'El estado actual del robot es {state}')
            # if state == 1 and euclD <= 0.2 and abs(anglD) <= 0.1 :
               # print ('Close Enough')  
               # result.result.success = 1
               # break

            # if i == 1000:
                # print (euclD)
                # i = 0
        # pub_stop.publish()

        # if result.result.success != 1:
            # print('time is over')
            # print (result)
        

if __name__=="__main__":
    global listener , goal_nav_publish , pub_stop, NS
    rospy.init_node('pumas_navigation_actionlib_server')
    
    #pub = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=1)
    #pub2 = rospy.Publisher('/aa/Markov_NXT/', PointStamped, queue_size=1)  
    #pub3= rospy.Publisher('aa/Markov_route',MarkerArray,queue_size=1)
    #pub_goal= rospy.Publisher('/clicked_point',PointStamped,queue_size=1)


    goal_nav_publish = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
    listener = tf.TransformListener()
    pub_stop = rospy.Publisher('/navigation/stop', Empty, queue_size=10)
    print ('pumas nav action server available')
    # NS = nav_status()
    s = pumas_navServer()
    rospy.spin()
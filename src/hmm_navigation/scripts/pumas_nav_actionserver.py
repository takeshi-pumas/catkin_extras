#! /usr/bin/env python3

import tf
import tf2_ros
import numpy as np
import rospy
import actionlib
from hmm_navigation.msg import NavigateAction ,NavigateActionGoal,NavigateActionFeedback,NavigateActionResult
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PointStamped , PoseStamped
from visualization_msgs.msg import Marker , MarkerArray
from std_msgs.msg import Empty, String

import rospkg
import yaml
##################################################
#YAML reader
def read_yaml(known_locations_file = '/known_locations.yaml'):
    rospack = rospkg.RosPack()
    file_path = rospack.get_path('location_reacher') +'/config_files' + known_locations_file

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
class location_reacher():
    def __init__(self):
        #self.goal = PoseStamped()
        #self.goal.header.frame_id = 'map'
        self.positionXYT = [0.0, 0.0, 0.0]
        #self._location_sub = rospy.Subscriber(
        #    '/goal_location', String, self._callback)
    def _callback(self, msg):
        # print(msg)
        goal_msg = msg.data.casefold()
        succ,loc = match_location(goal_msg)
        if succ:
            self.positionXYT = loc[:3]
            #move_base(self.positionXYT[0]['x'],self.positionXYT[1]['y'],self.positionXYT[2]['theta'], time_out = 20)
            # self._fill_msg()
        else:
            print(loc)

##################################################


def pose2feedback(pose_robot,quat_robot,timeleft,euclD):
    feed = NavigateActionFeedback()
    feed.feedback.x_robot   = pose_robot[0]
    feed.feedback.y_robot   = pose_robot[1]
    euler= tf.transformations.euler_from_quaternion((quat_robot[0] ,quat_robot[1] ,quat_robot[2] ,quat_robot[3] )) 
    feed.feedback.yaw_robot = euler[2]
    feed.feedback.timeleft    = timeleft
    feed.feedback.euclD= euclD
    return feed


def goal_police(goal):
    #goal corrector
    goal_corrected = goal


    return goal_corrected
class pumas_navServer():

    def __init__(self):
        self.pumas_nav_server = actionlib.SimpleActionServer("navigate",NavigateAction, execute_cb=self.execute_cb, auto_start=False)
        self.pumas_nav_server.start()
    
  
    def execute_cb(self, goal):
        

        ##goal police
        #goal_corrected = goal_police(goal)

        #####
        print (goal)
        x,y,yaw=goal.x ,goal.y,goal.yaw
        known_loc = goal.known_location.data.casefold()

        if known_loc is not 'None':
            succ,loc = match_location(known_loc)
            if succ:
                XYT = loc[:3]
                x,y,yaw = XYT[0]['x'],XYT[1]['y'],XYT[2]['theta']


        success = True
        result = NavigateActionResult()
        rate = rospy.Rate(10)
        timeout= rospy.Time.now().to_sec()+goal.timeout
        
        rot=tf.transformations.quaternion_from_euler(0,0,yaw)
        goal_pose= PoseStamped()
        goal_pose.pose.position.x=x
        goal_pose.pose.position.y=y
        goal_pose.pose.orientation.x=rot[0]
        goal_pose.pose.orientation.y=rot[1]
        goal_pose.pose.orientation.z=rot[2]
        goal_pose.pose.orientation.w=rot[3]
        goal_nav_publish.publish(goal_pose)


   
        result.result.success=2
        i=0
        while timeout >= rospy.Time.now().to_sec():     
            i+=1
            
            
            
            pose_robot,quat_robot=listener.lookupTransform('/map', 'base_footprint', rospy.Time(0)) 
            
            euclD=   np.linalg.norm(np.asarray((x,y))- pose_robot[:2])
        
            timeleft=timeout-rospy.Time.now().to_sec()  
            print (timeleft,'timeleft')   
            feed = pose2feedback(pose_robot,quat_robot,timeleft,euclD)
            self.pumas_nav_server.publish_feedback(feed.feedback)
        
            #if euclD<=0.2:
            #    print ('Close Enough')  
            #    result.result.success=1
                
            #    break
            if i ==1000:
                
                print (euclD)
                i=0
        pub_stop.publish()
        
        

            
        
        if result.result.success!=1:
            print('time is over')
            print (result)
        self.pumas_nav_server.set_succeeded(result.result)

            

        












        
if __name__=="__main__":
    global listener , goal_nav_publish , pub_stop
    rospy.init_node('pumas_navigation_actionlib_server')
    
    goal_nav_publish = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
    #pub = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=1)
    #pub2 = rospy.Publisher('/aa/Markov_NXT/', PointStamped, queue_size=1)  
    #pub3= rospy.Publisher('aa/Markov_route',MarkerArray,queue_size=1)
    #pub_goal= rospy.Publisher('/clicked_point',PointStamped,queue_size=1)
    listener = tf.TransformListener()
    pub_stop = rospy.Publisher('/navigation/stop', Empty, queue_size=10)
    print ('pumas nav action server available')
    s = pumas_navServer()
    rospy.spin()

    
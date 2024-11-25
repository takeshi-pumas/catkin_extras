#!/usr/bin/env python3
from smach_utils2 import *
from smach_ros import SimpleActionState
from action_server.msg import GraspAction
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


navclient = actionlib.SimpleActionClient('/move_base/move', MoveBaseAction)  #TOYOTA NAV

def move_base_toyota(goal_x,goal_y,goal_yaw,time_out=10):

    
    
    
    #using nav client and toyota navigation go to x,y,yaw
    #To Do: PUMAS NAVIGATION
    pose = PoseStamped()
    pose.header.stamp = rospy.Time(0)
    pose.header.frame_id = "map"
    pose.pose.position = Point(goal_x, goal_y, 0)
    quat = tf.transformations.quaternion_from_euler(0, 0, goal_yaw)
    pose.pose.orientation = Quaternion(*quat)


    # create a MOVE BASE GOAL
    goal = MoveBaseGoal()
    goal.target_pose = pose

    # send message to the action server
    navclient.send_goal(goal)

    # wait for the action server to complete the order
    navclient.wait_for_result(timeout=rospy.Duration(time_out))

    # print result of navigation
    action_state = navclient.get_state()
    return navclient.get_state()




class Initial(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0
    def execute(self, userdata):        
        rospy.loginfo('STATE : INITIAL')
        print('Robot neutral pose')
        return 'succ'
        
        if self.tries == 3:
            return 'tries'        
#######################################
class Choose_room(smach.State):
    def __init__(self):
        smach.State.__init__(self,output_keys=['room'], outcomes=['succ', 'failed'])
        self.tries = 0
    def execute(self, userdata):       
        self.tries+=1 
        print(f'STATE : CHOOSE_ROOM {self.tries}')
      
        global known_locs
        
        known_locs=yaml_to_df("/known_locations_storingTMR.yaml")

        random_choice = np.random.choice(known_locs['child_id_frame'])
        
        print (f'random destination is {random_choice}')
        userdata.room=random_choice
        return 'succ'
        
#####################################
class Goto_room(smach.State):
    def __init__(self):
        smach.State.__init__(self,input_keys=['room'], outcomes=['succ', 'failed', 'end'])
    def execute(self, userdata):        
        rospy.loginfo('STATE : GOTO_ROOM')
        print (f'userdata.room{userdata.room}')
        goal=known_locs[known_locs['child_id_frame'] == userdata.room][['x', 'y', 'th']].values[0]
        move_base_toyota(goal[0],goal[1],goal[2], 60)
        #res = omni_base.move_base(known_location=userdata.room, time_out=40)
        #print (f'res{res}')
        return 'succ'
             
              

# --------------------------------------------------
def init(node_name):
       print('smach ready')
   # --------------------------------------------------
# Entry point
if __name__ == '__main__':
    print("Takeshi STATE MACHINE...")
    init("takeshi_smach")
    # State machine, final state "END"
    sm = smach.StateMachine(outcomes=['END'])
    # sm.userdata.clear = False
    sis = smach_ros.IntrospectionServer('SMACH_VIEW_SERVER', sm, '/SM_STORING')
    sis.start()
    with sm:
        # State machine STICKLER
        smach.StateMachine.add("INITIAL",           Initial(),          transitions={'failed': 'INITIAL',           
                                                                                         'succ': 'CHOOSE_ROOM',   
                                                                                         'tries': 'END'})
        
        smach.StateMachine.add("CHOOSE_ROOM",           Choose_room(),          transitions={'failed': 'INITIAL',           
                                                                                         'succ': 'GOTO_ROOM'})
        smach.StateMachine.add("GOTO_ROOM",           Goto_room(),          transitions={'failed': 'GOTO_ROOM',           
                                                                                         'succ': 'CHOOSE_ROOM',   
                                                                                         'end': 'END'})


    outcome = sm.execute()

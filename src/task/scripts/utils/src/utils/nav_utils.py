# -*- coding: utf-8 -*-
from utils.misc_utils import *


class OMNIBASE():
    def __init__(self):
        self._base_vel_pub = rospy.Publisher(
            '/hsrb/command_velocity', Twist, queue_size=10)
        self.velX = 0
        self.velY = 0
        self.velT = 0
        self.timeout = 0.5
        self.MAX_VEL = 0.03
        self.MAX_VEL_THETA = 0.5
        self.navclient = actionlib.SimpleActionClient(
           '/navigate', NavigateAction)  # PUMAS NAV ACTION LIB
        self._tf_man = TF_MANAGER()

        self.nav_goal_pub = rospy.Publisher(
            '/move_base_simple/goal', PoseStamped, queue_size=10)

    def _move_base_vel(self):
        twist = Twist()
        twist.linear.x = self.velX
        twist.linear.y = self.velY
        twist.angular.z = self.velT
        self._base_vel_pub.publish(twist)

    def _move_base_time(self):
        start_time = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - start_time < self.timeout:
            self._move_base_vel()
    # Move base using relative movements

    def tiny_move(self, velX=0, velY=0, velT=0, std_time=0.5, MAX_VEL=0.03, MAX_VEL_THETA=0.5):
        self.MAX_VEL = MAX_VEL
        self.MAX_VEL_THETA = MAX_VEL_THETA
        self.timeout = std_time
        if abs(velX) > MAX_VEL:
            self.velX = MAX_VEL * (velX / abs(velX))
        else:
            self.velX = velX
        if abs(velY) > MAX_VEL:
            self.velY = MAX_VEL * (velY / abs(velY))
        else:
            self.velY = velY
        if abs(velT) > MAX_VEL_THETA:
            self.velT = MAX_VEL_THETA * (velT / abs(velT))
        else:
            self.velT = velT
        self._move_base_time()
    # Move base using navigation (absolute movement)

    def move_d_to(self, target_distance=0.5, target_link='None'):
        # Face towards Targetlink and get target distance close
        if target_link != 'None':
            target, _ = self._tf_man.getTF(target_frame=target_link)
            robot, _ = self._tf_man.getTF(target_frame='base_link')
            pose, _ = self._tf_man.getTF(
                target_frame=target_link, ref_frame='base_link')
        else:
            return False
        delta = np.asarray(target) - np.asarray(robot)
        dist = np.linalg.norm(delta)
        theta_goal = np.arctan2(delta[1], delta[0])
        x_goal = target[0] - target_distance * np.cos(theta_goal)
        y_goal = target[1] - target_distance * np.sin(theta_goal)
        succ = self.move_base(
            goal_x=x_goal, goal_y=y_goal, goal_yaw=theta_goal)
        return True

    def move_base(self, goal_x=0.0, goal_y=0.0, goal_yaw=0.0, time_out=30, known_location='None'):
        # Create and fill Navigate Action Goal message
        nav_goal = NavigateActionGoal()
        nav_goal.goal.x = goal_x
        nav_goal.goal.y = goal_y
        nav_goal.goal.yaw = goal_yaw
        nav_goal.goal.timeout = time_out
        nav_goal.goal.known_location = known_location
        print(nav_goal)

        # send message to the action server
        self.navclient.send_goal(nav_goal.goal)

        # wait for the action server to complete the order
        self.navclient.wait_for_result(timeout=rospy.Duration(time_out))
        # Results of navigation
        # PENDING         = 0
        # ACTIVE          = 1
        # PREEMPTED       = 2
        # SUCCEEDED       = 3
        # ABORTED         = 4
        # REJECTED        = 5
        # PREEMPTING      = 6
        # RECALLING       = 7
        # RECALLED        = 8
        # LOST            = 9
        action_state = self.navclient.get_state()
        return action_state

    def navigate(self, goal_x = 0.0, goal_y = 0.0, goal_theta = 0.0, known_location = 'None'):
        if known_location != 'None':
            x,y,theta = get_known_location(known_location)
        else:
            x,y,theta = goal_x,goal_y,goal_theta

        goal_position = Point(x,y,0.0)
        goal_orientation = Quaternion(*tf.transformations.quaternion_from_euler(0.0, 0.0, theta))
        goal_msg = PoseStamped()
        goal_msg.pose.position = goal_position
        goal_msg.pose.orientation = goal_orientation
        self.nav_goal_pub.publish(goal_msg)

    def _read_yaml(self, known_locations_file='/known_locations.yaml'):
        rospack = rospkg.RosPack()
        file_path = rospack.get_path('config_files') + known_locations_file

        with open(file_path, 'r') as file:
            content = yaml.safe_load(file)
        return content


    def _match_location(self, location):
        content = self._read_yaml()
        try:
            return True, content[location]
        except:
            return False, 'No location found'

    def get_known_location(self, location):
        succ, loc = self._match_location(known_loc)
        if succ:
            XYT = loc[:3]
            # it could change
            x, y, theta = XYT[0]['x'], XYT[1]['y'], XYT[2]['theta']
            return x, y, theta




class NAVIGATION():
    def __init__(self):
        self._base_vel_pub = rospy.Publisher(
            '/hsrb/command_velocity', Twist, queue_size=10)
        self.velX = 0
        self.velY = 0
        self.velT = 0
        self.timeout = 0.5
        self.MAX_VEL = 0.03
        self.MAX_VEL_THETA = 0.5
        self._tf_man = TF_MANAGER()

        self.nav_goal_pub = rospy.Publisher(
            '/move_base_simple/goal', PoseStamped, queue_size=10)
        self.nav_status_sub = rospy.Subscriber('/navigation/status', GoalStatus, self._nav_status_cb)
        self.arrived = False
        self.failed = False

    def _nav_status_cb(self, msg):
        print("Navigation status: ", msg.status)
        self.arrived = msg.status == 3
        self.failed = msg.status == 4

    #--------No navigation movement------------
    def _move_base_vel(self):
        twist = Twist()
        twist.linear.x = self.velX
        twist.linear.y = self.velY
        twist.angular.z = self.velT
        self._base_vel_pub.publish(twist)

    def _move_base_time(self):
        start_time = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - start_time < self.timeout:
            self._move_base_vel()
    

    def tiny_move(self, velX=0, velY=0, velT=0, std_time=0.5, MAX_VEL=0.03, MAX_VEL_THETA=0.5):
        self.MAX_VEL = MAX_VEL
        self.MAX_VEL_THETA = MAX_VEL_THETA
        self.timeout = std_time
        if abs(velX) > MAX_VEL:
            self.velX = MAX_VEL * (velX / abs(velX))
        else:
            self.velX = velX
        if abs(velY) > MAX_VEL:
            self.velY = MAX_VEL * (velY / abs(velY))
        else:
            self.velY = velY
        if abs(velT) > MAX_VEL_THETA:
            self.velT = MAX_VEL_THETA * (velT / abs(velT))
        else:
            self.velT = velT
        self._move_base_time()
    #-------------------------------------------

    # Move base using navigation (absolute movement)

    def move_d_to(self, target_distance=0.5, target_link='None'):
        # Face towards Targetlink and get target distance close
        if target_link != 'None':
            target, _ = self._tf_man.getTF(target_frame=target_link)
            robot, _ = self._tf_man.getTF(target_frame='base_link')
            pose, _ = self._tf_man.getTF(
                target_frame=target_link, ref_frame='base_link')
        else:
            return False
        delta = np.asarray(target) - np.asarray(robot)
        dist = np.linalg.norm(delta)
        theta_goal = np.arctan2(delta[1], delta[0])
        x_goal = target[0] - target_distance * np.cos(theta_goal)
        y_goal = target[1] - target_distance * np.sin(theta_goal)
        succ = self.move_base(
            goal_x=x_goal, goal_y=y_goal, goal_theta=theta_goal)
        return True

    def _fill_nav_request(x,y,theta):
        position = Point(x,y,0.0)
        orientation = Quaternion(*tf.transformations.quaternion_from_euler(0.0, 0.0, theta))
        msg = PoseStamped()
        msg.pose.position = position
        msg.pose.orientation = orientation
        return msg

    def move_base(self, goal_x = 0.0, goal_y = 0.0, goal_theta = 0.0, known_location = 'None', timeout=30):
        #process new request
        self.arrived = False
        self.failed = False
        if known_location != 'None':
            x, y, theta = self.get_known_location(known_location)
        else:
            x, y, theta = goal_x, goal_y, goal_theta

        #fill message and publish it
        goal_msg = self._fill_nav_request(x, y, theta)
        self.nav_goal_pub.publish(goal_msg)
        
        rate = rospy.Rate(10)
        start_time = rospy.Time.now()

        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            elapsed_time = current_time - start_time
            if self.arrived:
                return True
            elif self.failed:
                return False
            elif elapsed_time.to_sec() > timeout:
                return False
            rate.sleep()

    #-----Known location parser-------
    def _read_yaml(self, known_locations_file='/known_locations.yaml'):
        rospack = rospkg.RosPack()
        file_path = rospack.get_path('config_files') + known_locations_file

        with open(file_path, 'r') as file:
            content = yaml.safe_load(file)
        return content


    def _match_location(self, location):
        content = self._read_yaml()
        try:
            return True, content[location]
        except:
            return False, 'No location found'

    def get_known_location(self, location):
        succ, loc = self._match_location(location)
        if succ:
            XYT = loc[:3]
            # it could change
            x, y, theta = XYT[0]['x'], XYT[1]['y'], XYT[2]['theta']
            return x, y, theta
    #------------------------------------
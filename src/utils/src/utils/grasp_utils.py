# -*- coding: utf-8 -*-
from utils.misc_utils import *
from utils.nav_utils import OMNIBASE


#Class to get wrist sensor info (Force and torque)
class WRIST_SENSOR:
    def __init__(self, topic_wrench_sensor = '/hsrb/wrist_wrench/compensated'):
        self._cam_sub = rospy.Subscriber(topic_wrench_sensor,
            WrenchStamped, self._callback)
        self.force = None
        self.torque = None
         
    def _callback(self, msg):
        self.force = msg.wrench.force
        self.torque = msg.wrench.torque

    def get_force(self):
        force = [self.force.x, self.force.y, self.force.z]
        return force
    
    def get_torque(self):
        torque = [self.torque.x, self.torque.y, self.torque.z]
        return torque

#Class to handle end effector (gripper)
class GRIPPER:
    def __init__(self):
        self._grip_cmd_pub = rospy.Publisher(
            '/hsrb/gripper_controller/command',
            trajectory_msgs.msg.JointTrajectory, queue_size=100)
        self._grip_cmd_force = rospy.Publisher(
            '/hsrb/gripper_controller/grasp/goal',
            tmc_control_msgs.msg.GripperApplyEffortActionGoal, queue_size=100)
                    
        self._joint_name = "hand_motor_joint"
        self._position = 0.5
        self._velocity = 0.5
        self._effort = 0.0
        self._duration = 1

    def _manipulate_gripper(self):
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names = [self._joint_name]
        p = trajectory_msgs.msg.JointTrajectoryPoint()
        p.positions = [self._position]
        p.velocities = [self._velocity]
        p.accelerations = []
        p.effort = [self._effort]
        p.time_from_start = rospy.Duration(self._duration)
        traj.points = [p]
        self._grip_cmd_pub.publish(traj)
        
    def _apply_force(self, force = 0.5):
        app_force = tmc_control_msgs.msg.GripperApplyEffortActionGoal()
        app_force.goal.effort = -force
        self._grip_cmd_force.publish(app_force)
        
    def change_velocity(self, newVel: int):
        self._velocity = newVel
    
    def open(self):
        self._position = 1.23
        self._effort = 0
        self._manipulate_gripper()

    def steady(self):
        self._position = 0.2
        self._effort = -0.3
        self._manipulate_gripper()
        
    def close(self, force = 0.5):
        self._position = 0.0
        self._effort = 0.3
        # self._manipulate_gripper()
        self._apply_force(force)
        rospy.sleep(0.8)



class GAZE:
    def __init__(self, head_controller_topic = '/hsrb/head_trajectory_controller/command'):
        self._x = 0
        self._y = 0
        self._z = 0
        self._reference = 'map'
        self._cam = 'head_rgbd_sensor_link'
        self._base = 'base_link'
        self._hand = 'hand_palm_link'
        self._tf_man = TF_MANAGER()
        self._pub = rospy.Publisher( head_controller_topic,
            trajectory_msgs.msg.JointTrajectory, queue_size=10)
    def _gaze_point(self):
    ###Moves head to make center point of rgbd image to coordinates w.r.t.map
        trans,_ = self._tf_man.getTF(ref_frame=self._reference,target_frame=self._cam)
        rospy.sleep(0.3)
        _,rot = self._tf_man.getTF(ref_frame=self._reference, target_frame=self._base)
        _,_, th_rob = tf.transformations.euler_from_quaternion(rot)
        
        x_rob, y_rob, z_rob = trans[0], trans[1], trans[2]
        D_x = x_rob - self._x
        D_y = y_rob - self._y
        D_z = z_rob - self._z
        D_th = np.arctan2(D_y,D_x)
        #pan: horizontal angle, tilt: vertical angle
        #keep pan angle between -2*pi and 2*pi
        pan_correct = (- th_rob + D_th + np.pi) % (2*np.pi)
        #pan mapping from pi and -pi
        if pan_correct > np.pi:
            pan_correct -= 2 * np.pi
        elif pan_correct < -np.pi:
            pan_correct += 2 * np.pi
        tilt_correct = -np.arctan2(D_z,np.linalg.norm((D_x,D_y)))
        #pan exorcist alert 
        if abs(pan_correct) > 0.5 * np.pi:
            print ('Exorcist alert')
            # pan_correct=0.5*np.pi
            self.set_joint_values([0.0, tilt_correct])
            self.turn_base_gaze()
            return [0.0, tilt_correct]
            # return self._gaze_point()
        else:    
            head_pose = [pan_correct,  tilt_correct]
            return head_pose
    
    def _gaze_abs_rel(self, x, y, z):
        self._x = x
        self._y = y
        self._z = z
        head_pose =  self._gaze_point()
        self.set_joint_values(head_pose)
        return head_pose

    def absolute(self, x: float, y:float, z: float):
        #Head gaze to a x, y, z point relative to map
        self._reference = 'map'
        return self._gaze_abs_rel(x,y,z)

    def relative(self, x:float, y:float, z:float):
        #Head gaze to a x, y, z point relative to base_link
        self._reference = 'base_link'
        return self._gaze_abs_rel(x,y,z)
        
    def set_named_target(self, pose='neutral'):
        if pose == 'down':
            head_pose = [0.0,-1.0]
        elif pose == 'up':
            head_pose = [0.0, 1.0]
        elif pose == 'right':
            head_pose = [0.7, 0.0]
        elif pose == 'left':
            head_pose = [-0.7, 0.0]
        else:
            head_pose = [0.0, 0.0]
        self.set_joint_values(head_pose) 

    def get_joint_values(self):
        states = rospy.wait_for_message('/hsrb/joint_states', JointState)
        st = states.position
        return [st[9], st[10]]

    def set_joint_values(self, head_pose = [0.0,0.0]):
        # fill ROS message
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names = ["head_pan_joint", "head_tilt_joint"]
        p = trajectory_msgs.msg.JointTrajectoryPoint()
        p.positions = head_pose
        p.velocities = [0.1, 0.1]
        p.time_from_start = rospy.Duration(0.07)
        traj.points = [p]

        # publish ROS message
        self._pub.publish(traj)

    def to_tf(self, target_frame='None'):
        if target_frame != 'None':
            rospy.sleep(0.5)
            xyz,_ = self._tf_man.getTF(target_frame=target_frame)
            rospy.sleep(0.8)
            tries = 1
            #type(xyz) is bool --> tf no encontrada
            while (type(xyz) is bool): #and (tries <= 5):
                #tries += 1
                #print(tries)
                #xyz,_ = self._tf_man.getTF(target_frame=target_frame)
                #rospy.sleep(0.8)
                num_loc = (int) (target_frame.replace('Place_face', ''))
                locs = [[9.65,-2.02,0.0], [9.72,-2.1,0.0], [9.67,-3.06,0.0]]
                loc =locs[num_loc-1]
                xyz = [loc[0], loc[1], 0.85]
                #for i, loc in enumerate(locs):
                #    pos = [loc[0], loc[1], 0.85]
            if type(xyz) is not bool:
               self.absolute(*xyz)

    def turn_base_gaze(self,tf='None', to_gaze = 'base_link'):
        base = OMNIBASE()
        succ = False
        THRESHOLD = 0.05
        tries = 0
        if tf != 'None':
            target_frame = tf
        else:
            target_frame = 'gaze'
            self._tf_man.pub_static_tf(pos=[self._x,self._y,self._z], point_name='gaze')
            rospy.sleep(0.1)
        while (not succ) and (tries <=10):
            tries += 1
            rospy.sleep(0.2)
            xyz,_=self._tf_man.getTF(target_frame=target_frame, ref_frame=to_gaze)
            eT = 0
            if type(xyz) is not bool:
                eT = np.arctan2(xyz[1],xyz[0])
                succ = abs(eT) < THRESHOLD 
            if succ:
                eT = 0
            base.tiny_move(velT = eT, MAX_VEL_THETA=1.1)
        return True

def set_pose_goal(pos=[0,0,0], rot=[0,0,0,1]):
    pose_goal = Pose()
    pose_goal.position = Point(*pos)
    pose_goal.orientation = Quaternion(*rot)
    return pose_goal


class ARM:
    def __init__(self, joint_names = ["arm_lift_joint", "arm_flex_joint", "arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"],
                 arm_controller_action_client = '/hsrb/arm_trajectory_controller/follow_joint_trajectory'):
        # import tf2_ros
        self._joint_names = joint_names
        self._cli = actionlib.SimpleActionClient(arm_controller_action_client,
            control_msgs.msg.FollowJointTrajectoryAction)
        self._tf_man = TF_MANAGER()
        self._grasp_base = OMNIBASE()
        self._wrist = WRIST_SENSOR()

    def set_joint_values(self, joint_values = [0.0, 0.0, -1.6, -1.6, 0.0]):
        goal = control_msgs.msg.FollowJointTrajectoryGoal()
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names = self._joint_names
        p = trajectory_msgs.msg.JointTrajectoryPoint()
        p.positions = joint_values
        p.velocities = [0.1, 0.1, 0.1, 0.1, 0.1]
        p.time_from_start = rospy.Duration(0.5)
        traj.points = [p]
        goal.trajectory = traj

        # send message to the action server
        #print('message sent')
        self._cli.send_goal(goal)

        # wait for the action server to complete the order
        return self._cli.wait_for_result()
    
    def get_joint_values(self):
        states = rospy.wait_for_message('/hsrb/joint_states', JointState)
        st = states.position
        return [st[1], st[0],st[2], st[11], st[12]]

    def set_named_target(self, pose = 'go'):
        if pose == 'neutral':
            joint_values = [0.0, 0.0, 0.0, -1.6, 0.0]
        elif pose == 'grasp_floor':
            joint_values = [0.27, -2.09, 0.0, -1.12, 0.0]
        #go case
        else:   
            joint_values = [0.0, 0.0, -1.6, -1.6, 0.0]
        self.set_joint_values(joint_values)

    def check_grasp(self, weight = 1.0):
        
        force = self._wrist.get_force()
        force = np.linalg.norm(np.array(force))
        if force > weight:
            return True
        else:
            l_finger = 'hand_l_finger_tip_frame'
            r_finger = 'hand_r_finger_tip_frame'
            dist,_ = self._tf_man.getTF(target_frame=r_finger, ref_frame=l_finger)
            np.array(dist)
            dist = np.linalg.norm(dist)
            if dist > 0.02:
                return True
            else:
                return False
            
    def move_hand_to_target(self, target_frame = 'target', offset=[0,0,0], THRESHOLD = 0.05):
        succ = False
        #THRESHOLD = 0.05
        #THRESHOLD_T = 0.1

        hand = 'hand_palm_link'
        base = 'base_link'

        while not succ and not rospy.is_shutdown():
            try:
                target_base, _ = self._tf_man.getTF(target_frame = hand, ref_frame=base)
                dist, _ = self._tf_man.getTF(target_frame = target_frame, ref_frame=base)
                e = np.array(dist) - np.array(target_base) - np.array(offset)
                e[2] = 0
                
                rospy.loginfo("Distance to goal: {:.2f}, {:.2f}".format(e[0], e[1]))
                if abs(e[0]) < THRESHOLD:
                    e[0] = 0
                if abs(e[1]) < THRESHOLD:
                    e[1] = 0
                #if abs(eT) < THRESHOLD_T:
                #    eT = 0
                e_norm = np.linalg.norm(e)
                succ = e_norm < THRESHOLD #and abs(eT) < THRESHOLD_T
                #grasp_base.tiny_move(velX=0.4*e[0], velY=0.6*e[1], velT=0.7*eT,  std_time=0.2, MAX_VEL=0.3, MAX_VEL_THETA=0.9)
                self._grasp_base.tiny_move(velX=0.4*e[0], velY=0.6*e[1], std_time=0.2, MAX_VEL=0.3)
            except (tf2.LookupException, tf2.ConnectivityException, tf2.ExtrapolationException) as e:
                rospy.logwarn("Failed to get transform: {}".format(str(e)))
                continue
        return succ


from geometry_msgs.msg import Twist
import math

class BASE:
    def __init__(self, cmd_vel_topic='/hsrb/command_velocity'):
        self._base_vel_pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
        self.velX = 0
        self.velY = 0
        self.velT = 0
        self.timeout = 0.5
        self.MAX_VEL = 0.03
        self.MAX_VEL_THETA = 0.5

    def _move_base_vel(self):
        twist = Twist()
        twist.linear.x = self.velX
        twist.linear.y = self.velY
        twist.angular.z = self.velT
        self._base_vel_pub.publish(twist)

    def _move_base_time(self):
        start_time = rospy.Time.now().to_sec()
        end_time = start_time + self.timeout
        while rospy.Time.now().to_sec() < end_time and not rospy.is_shutdown():
            self._move_base_vel()

    def tiny_move(self, velX=0, velY=0, velT=0, std_time=0.5, MAX_VEL=0.03, MAX_VEL_THETA=0.5):
        self.MAX_VEL = MAX_VEL
        self.MAX_VEL_THETA = MAX_VEL_THETA
        self.timeout = std_time
        
        # Limit velocities
        self.velX = self.limit_velocity(velX, MAX_VEL)
        self.velY = self.limit_velocity(velY, MAX_VEL)
        self.velT = self.limit_velocity(velT, MAX_VEL_THETA)
        
        self._move_base_time()
    
    def limit_velocity(self, velocity, max_velocity):
        return max_velocity * math.copysign(1, velocity) if abs(velocity) > max_velocity else velocity
    
    def turn_radians(self, time, radians):
        vel_theta = radians / time
        self.tiny_move(velT= vel_theta, std_time= time, MAX_VEL_THETA= vel_theta)

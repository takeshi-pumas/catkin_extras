# -*- coding: utf-8 -*-
from utils.misc_utils import *
from utils.nav_utils import OMNIBASE


#Class to get wrist sensor info (Force and torque)
class WRIST_SENSOR():
    def __init__(self):
        self._cam_sub = rospy.Subscriber(
            '/hsrb/wrist_wrench/compensated',
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
class GRIPPER():
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
        
    def _apply_force(self):
        app_force = tmc_control_msgs.msg.GripperApplyEffortActionGoal()
        app_force.goal.effort = -0.5
        self._grip_cmd_force.publish(app_force)
        
    def change_velocity(self, newVel):
        self._velocity = newVel
    
    def open(self):
        self._position = 1.23
        self._effort = 0
        self._manipulate_gripper()

    def steady(self):
        self._position = 0.2
        self._effort = -0.3
        self._manipulate_gripper()
        
    def close(self):
        self._position = 0.0
        self._effort = 0.3
        # self._manipulate_gripper()
        self._apply_force()
        rospy.sleep(0.8)



class GAZE():
    def __init__(self):
        self._x = 0
        self._y = 0
        self._z = 0
        self._reference = 'map'
        self._cam = 'head_rgbd_sensor_link'
        self._base = 'base_link'
        self._hand = 'hand_palm_link'
        self._tf_man = TF_MANAGER()
        self._pub = rospy.Publisher(
            '/hsrb/head_trajectory_controller/command',
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

    def absolute(self, x, y, z):
        #Head gaze to a x, y, z point relative to map
        self._reference = 'map'
        return self._gaze_abs_rel(x,y,z)

    def relative(self, x, y, z):
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
            xyz,_ = self._tf_man.getTF(target_frame=target_frame)
            rospy.sleep(0.5)
            tries = 1
            while type(xyz) is bool or tries <= 5:
                tries += 1
                xyz,_ = self._tf_man.getTF(target_frame=target_frame)
                rospy.sleep(0.3)
            if type(xyz) is not bool:
                self.absolute(*xyz)

    def turn_base_gaze(self,tf='None'):
        base = OMNIBASE()
        succ = False
        THRESHOLD = 0.05
        if tf != 'None':
            target_frame = tf
        else:
            target_frame = 'gaze'
            self._tf_man.pub_static_tf(pos=[self._x,self._y,self._z], point_name='gaze')
        while not succ:
            xyz,_=self._tf_man.getTF(target_frame=target_frame, ref_frame=self._base)
            eT = np.arctan2(xyz[1],xyz[0])
            succ = abs(eT) < THRESHOLD 
            if succ:
                eT = 0
            base.tiny_move(velT = eT)
        return True


def set_pose_goal(pos=[0,0,0], rot=[0,0,0,1]):
    pose_goal = Pose()
    pose_goal.position = Point(*pos)
    pose_goal.orientation = Quaternion(*rot)
    return pose_goal


class ARM():
    def __init__(self):
        import tf2_ros
        self._joint_names = ["arm_lift_joint", "arm_flex_joint", "arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
        self._cli = actionlib.SimpleActionClient(
            '/hsrb/arm_trajectory_controller/follow_joint_trajectory',
            control_msgs.msg.FollowJointTrajectoryAction)
        self._tf_man = TF_MANAGER()
        self._grasp_base = OMNIBASE()
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
        wrist=WRIST_SENSOR()

        force = wrist.get_force()
        force = np.array(force)
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

        while not succ:
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
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logwarn("Failed to get transform: {}".format(str(e)))
                continue
        return succ

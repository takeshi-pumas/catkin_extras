# -*- coding: utf-8 -*-
from utils.misc_utils import *



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
        #x_rob, y_rob, z_rob = *trans
        D_x = x_rob - self._x
        D_y = y_rob - self._y
        D_z = z_rob - self._z
        D_th = np.arctan2(D_y,D_x)
        pan_correct = (- th_rob + D_th + np.pi) % (2*np.pi)
        if(pan_correct > np.pi):
            pan_correct -= 2*np.pi
        if(pan_correct < -np.pi):
            pan_correct += 2*np.pi
        if ((pan_correct) > .5 * np.pi):
            print ('Exorcist alert')
            pan_correct=.5*np.pi
            
        head_pose = [0,0]
        head_pose[0] = pan_correct
        tilt_correct = np.arctan2(D_z,np.linalg.norm((D_x,D_y)))
        head_pose [1] = -tilt_correct
        return head_pose
    
    def absolute(self, x, y, z):
        #Head gaze to a x, y, z point relative to map
        self._x = x
        self._y = y
        self._z = z
        self._reference = 'map'
        head_pose =  self._gaze_point()
        self.set_joint_values(head_pose)
        return head_pose
    def relative(self, x, y, z):
        #Head gaze to a x, y, z point relative to base_link
        self._x = x
        self._y = y
        self._z = z
        self._reference = 'base_link'
        head_pose =  self._gaze_point()
        self.set_joint_values(head_pose)
        return head_pose
    def set_joint_values(self, head_pose):
        # fill ROS message
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names = ["head_pan_joint", "head_tilt_joint"]
        p = trajectory_msgs.msg.JointTrajectoryPoint()
        p.positions = head_pose
        p.velocities = [0, 0]
        p.time_from_start = rospy.Duration(3)
        traj.points = [p]

        # publish ROS message
        self._pub.publish(traj)
        
    def to_tf(self, target_frame='None'):
        if target_frame is not 'None':
            xyz,_ = self._tf_man.getTF(target_frame=target_frame)
            self.absolute(*xyz)

def set_pose_goal(pos=[0,0,0], rot=[0,0,0,1]):
    pose_goal = Pose()
    pose_goal.position = Point(*pos)
    pose_goal.orientation = Quaternion(*rot)
    return pose_goal
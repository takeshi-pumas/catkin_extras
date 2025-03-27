import rospy
from typing import List, Tuple, Optional, Literal
import numpy as np
import tf2_ros
from tf import transformations
import actionlib

from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState
from tmc_msgs.msg import Voice 
from tmc_control_msgs.msg import GripperApplyEffortActionGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal

from common.navigation_functions import Navigation
from common.ros_functions import TFManager


# HSR functionalities

class Talker:
    def __init__(self, talk_request_topic='/talk_request'):
        """Initialize the talker with ROS publisher"""
        try:
            self.talk_pub = rospy.Publisher(talk_request_topic, Voice, queue_size=10)
        except Exception as e:
            rospy.logerr(f"Failed to initialize Voice: {e}")
            raise
    @staticmethod
    def _fillMsg(msg):
        """Create a Voice message"""
        voice_msg = Voice()
        voice_msg.interrupting = False
        voice_msg.queueing = False
        voice_msg.language = 0
        voice_msg.sentence = msg
        return voice_msg

    def talk(self, sentence: str, timeout: float = 0.0) -> bool:
        """
        Send a talk request and wait for result (using publisher)
        
        Args:
            sentence: Text to speak
            timeout: Time to wait for result in seconds
            
        Returns:
            bool: True if message was published, False otherwise
        """
        try:
            # Publicamos el mensaje en el tópico /talk_request
            voice_msg = self._fillMsg(sentence)
            self.talk_pub.publish(voice_msg)
            
            rospy.loginfo(f"Sent sentence: {sentence}")
            if timeout > 0.0:
                rospy.sleep(timeout)  # Esperamos por el tiempo que definimos
            
            return True  # Suponemos que el mensaje fue publicado correctamente
            
        except Exception as e:
            rospy.logwarn(f"Failed to send talk message: {e}")
            return False

class WristSensor:
    """Handles wrist force/torque sensor readings with proper cleanup"""
    
    def __init__(self, topic: str = '/hsrb/wrist_wrench/compensated'):
        self._force = None
        self._torque = None
        self._subscriber = rospy.Subscriber(
            topic,
            WrenchStamped, 
            self._callback,
            queue_size=1
        )

    def __del__(self):
        """Cleanup subscriber"""
        if hasattr(self, '_subscriber') and not rospy.is_shutdown():
            self._subscriber.unregister()

    def _callback(self, msg: WrenchStamped) -> None:
        """Store latest sensor readings"""
        self._force = [msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z]
        self._torque = [msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z]

    def get_force(self) -> Optional[List[float]]:
        """Get current force readings"""
        return self._force
    
    def get_torque(self) -> Optional[List[float]]:
        """Get current torque readings"""
        return self._torque

class Gripper:
    """Controls robot gripper with proper resource management"""
    
    def __init__(self):
        self._joint_name = "hand_motor_joint"
        self._grip_cmd_pub = rospy.Publisher(
            '/hsrb/gripper_controller/command',
            JointTrajectory, 
            queue_size=1
        )
        self._grip_force_pub = rospy.Publisher(
            '/hsrb/gripper_controller/grasp/goal',
            GripperApplyEffortActionGoal, 
            queue_size=1
        )

    def __del__(self):
        """Cleanup publishers"""
        if not rospy.is_shutdown():
            if hasattr(self, '_grip_cmd_pub'):
                self._grip_cmd_pub.unregister()
            if hasattr(self, '_grip_force_pub'):
                self._grip_force_pub.unregister()

    def _create_trajectory(self, position: float, velocity: float = 0.5, 
                         effort: float = 0.0, duration: float = 1.0) -> JointTrajectory:
        """Creates gripper trajectory message"""
        traj = JointTrajectory()
        traj.joint_names = [self._joint_name]
        point = JointTrajectoryPoint()
        point.positions = [position]
        point.velocities = [velocity]
        point.effort = [effort]
        point.time_from_start = rospy.Duration(duration)
        traj.points = [point]
        return traj

    def open(self) -> None:
        """Opens gripper fully"""
        traj = self._create_trajectory(1.23)
        self._grip_cmd_pub.publish(traj)

    def close(self, force: float = 0.5) -> None:
        """
        Closes gripper with specified force
        
        Args:
            force: Closing force (0.0-1.0)
        """
        force_msg = GripperApplyEffortActionGoal()
        force_msg.goal.effort = -abs(force)
        self._grip_force_pub.publish(force_msg)
        rospy.sleep(0.8)

    def steady(self) -> None:
        """Sets gripper to steady grip position"""
        traj = self._create_trajectory(0.2, effort=-0.3)
        self._grip_cmd_pub.publish(traj)


class Gaze:
    """Controls robot head gaze direction with pan and tilt movements"""

    def __init__(self, 
                 head_controller_topic: str = '/hsrb/head_trajectory_controller/command',
                 pan_limits: Tuple[float, float] = (-1.745, 1.745),  # Hardware limits (-3.839, 1.745)
                 tilt_limits: Tuple[float, float] = (-1.570, 0.523),
                 tf_timeout: float = 0.5):
        """
        Args:
            head_controller_topic: ROS topic for head controller
            pan_limits: Min/max pan angles in radians
            tilt_limits: Min/max tilt angles in radians 
            tf_timeout: Timeout for TF lookups
        """
        self._target_point = np.zeros(3)  # Target point to look at
        self._reference_frame = 'map'  # Reference frame for target point
        
        # TF frames
        self._camera_frame = 'head_rgbd_sensor_link'
        self._base_frame = 'base_link'
        
        # Movement limits
        self._pan_limits = pan_limits
        self._tilt_limits = tilt_limits
        self._tf_timeout = tf_timeout
        
        # Publishers
        self._head_pub = rospy.Publisher(
            head_controller_topic,
            JointTrajectory, 
            queue_size=1
        )
        self._base_turn_pub = rospy.Publisher(
            "simple_move/goal_dist_angle", 
            Float32MultiArray, 
            queue_size=1
        )

        # TF buffer
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

    def __del__(self):
        """Cleanup publishers"""
        if not rospy.is_shutdown():
            self._head_pub.unregister()
            self._base_turn_pub.unregister()

    def _get_transform(self, target_frame: str, source_frame: str) -> Optional[Tuple[List[float], List[float]]]:
        """Get transform between frames"""
        try:
            trans = self._tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rospy.Time(0),
                rospy.Duration(self._tf_timeout)
            )
            translation = [
                trans.transform.translation.x,
                trans.transform.translation.y,
                trans.transform.translation.z
            ]
            rotation = [
                trans.transform.rotation.x,
                trans.transform.rotation.y,
                trans.transform.rotation.z,
                trans.transform.rotation.w
            ]
            return translation, rotation
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"TF lookup failed: {e}")
            return None

    def _calculate_head_angles(self) -> Tuple[float, float]:
        """
        Calculate required pan and tilt angles to look at target.
        Returns absolute angles considering:
        - Robot's base orientation
        - Camera offset and rotation
        - Absolute encoder readings
        """
        # Get current poses
        camera_pose = self._get_transform(self._reference_frame, self._camera_frame)
        if not camera_pose:
            return 0.0, 0.0
            
        base_pose = self._get_transform(self._reference_frame, self._base_frame)
        if not base_pose:
            return 0.0, 0.0

        # Get positions and orientations
        camera_pos = np.array(camera_pose[0])
        base_pos = np.array(base_pose[0])
        
        # Get robot's base orientation (yaw)
        _, _, robot_yaw = transformations.euler_from_quaternion(base_pose[1])
        
        # Calculate vector from base to target (in world frame)
        target_vector = self._target_point - base_pos
        
        # Calculate desired angle in world frame
        target_yaw = np.arctan2(target_vector[1], target_vector[0])
        
        # Calculate pan angle relative to robot's current orientation
        # This gives us the absolute angle needed for the head
        pan = target_yaw - robot_yaw
        
        # Normalize pan to [-π, π]
        pan = np.arctan2(np.sin(pan), np.cos(pan))
        
        # Calculate vertical components considering camera height
        camera_height = camera_pos[2] - self._target_point[2]
        horizontal_distance = np.linalg.norm(target_vector[:2])
        
        # Calculate tilt angle using camera height
        # Negative because positive tilt points down in HSR
        tilt = -np.arctan2(camera_height, horizontal_distance)
        
        # Add offset for camera's mounting angle if needed
        # HSR's camera is mounted at a slight angle
        camera_tilt_offset = -0.1  # Approximate value, adjust if needed
        tilt += camera_tilt_offset
        

        
        return pan, tilt

    def move_head(self, pan: float, tilt: float) -> None:
        """Send movement command to head"""
        traj = JointTrajectory()
        traj.joint_names = ["head_pan_joint", "head_tilt_joint"]
        
        point = JointTrajectoryPoint()
        point.positions = [pan, tilt]
        point.velocities = [0.5, 0.5]  # Moderate velocity
        point.time_from_start = rospy.Duration(0.5)
        
        traj.points = [point]
        self._head_pub.publish(traj)

    def request_base_turn(self, angle: float) -> None:
        """
        Request base rotation relative to current robot position
        
        Args:
            angle: Desired rotation angle in radians (relative to current robot orientation)
        """
        # Normalize angle to [-π, π] for shortest rotation
        angle = np.arctan2(np.sin(angle), np.cos(angle))
        
        msg = Float32MultiArray()
        msg.data = [0.0, angle]  # [distance, relative_angle]
        self._base_turn_pub.publish(msg)
        rospy.sleep(1.0)  # Give time for base to start moving

    def look_at(self, x: float, y: float, z: float, 
                frame: Literal['map', 'base_link'] = 'map', 
                move_base: bool = True) -> bool:
        """
        Point head to look at specified point
        
        Args:
            x, y, z: Target point coordinates
            frame: Reference frame ('map' or 'base_link')
            move_base: Whether to rotate base for targets outside pan limits
            
        Returns:
            bool: Success status
        """
        
        try:
            self._target_point = np.array([x, y, z])
            self._reference_frame = frame
     
            pan, tilt = self._calculate_head_angles()
            
            # Check if target requires base movement
            if move_base and abs(pan) > abs(self._pan_limits[1]):
                excess_angle = pan - np.sign(pan) * self._pan_limits[1]
                self.request_base_turn(excess_angle)
                pan = np.sign(pan) * self._pan_limits[1]

                    # Clamp angles to hardware limits
            # pan = np.clip(pan, self._pan_limits[0], self._pan_limits[1])
            tilt = np.clip(tilt, self._tilt_limits[0], self._tilt_limits[1])
            
            self.move_head(pan, tilt)
            return True
            
        except Exception as e:
            rospy.logerr(f"Failed to execute head movement: {e}")
            return False

    def look_at_frame(self, frame_id: str) -> bool:
        """Look at a specific TF frame"""
        transform = self._get_transform(self._reference_frame, frame_id)
        if not transform:
            return False
            
        position = transform[0]
        return self.look_at(*position, frame=self._reference_frame)

    def set_named_target(self, pose: Literal['neutral', 'down', 'up', 'left', 'right'] = 'neutral') -> None:
        """Move head to predefined pose"""
        poses = {
            'neutral': (0.0, 0.0),
            'down': (0.0, -1.0),
            'up': (0.0, 0.5),
            'left': (-0.7, 0.0),
            'right': (0.7, 0.0)
        }
        self.move_head(*poses.get(pose, poses['neutral']))


# class WRIST_SENSOR:
#     def __init__(self, 
#         topic_wrench_sensor = '/hsrb/wrist_wrench/compensated'):
#         self.wrist_sensor_sub = rospy.Subscriber(topic_wrench_sensor,
#         WrenchStamped, self._wrist_callback)
#         self.force = None
#         self.torque = None
         
#     def _wrist_callback(self, msg):
#         self.force = [self.force.x, self.force.y, self.force.z]
#         self.torque = [self.torque.x, self.torque.y, self.torque.z]

#     def get_force(self):
#         force = [self.force.x, self.force.y, self.force.z]
#         return force
    
#     def get_torque(self):
#         torque = [self.torque.x, self.torque.y, self.torque.z]
#         return torque

class ArmController:
    """Controls HSR robot arm movements and grasping operations"""
    
    def __init__(self, 
                 base: Navigation,
                 tf_manager: TFManager,
                 joint_names: List[str] = None,
                 action_ns: str = '/hsrb/arm_trajectory_controller/follow_joint_trajectory'):
        """
        Initialize arm controller with existing instances of required utilities
        
        Args:
            base: Existing OMNIBASE instance
            tf_manager: Existing TF_MANAGER instance
            wrist_sensor: Existing WRIST_SENSOR instance
            joint_names: List of joint names to control
            action_ns: Action server namespace
        """
        # Store utility instances
        self._base = base
        self._tf_man = tf_manager
        # self._wrist = wrist_sensor
        
        # Joint configuration
        self._joint_names = joint_names or [
            "arm_lift_joint", "arm_flex_joint", "arm_roll_joint",
            "wrist_flex_joint", "wrist_roll_joint"
        ]
        
        # Named poses for common positions
        self._named_poses = {
            'neutral': [0.0, 0.0, 0.0, -1.6, 0.0],
            'go': [0.0, 0.0, -1.6, -1.6, 0.0],
            'grasp_floor': [0.27, -2.09, 0.0, -1.12, 0.0],
            'point': [0.03, -0.5, 0.0, -1.09, 0.0]
        }
        
        # Movement parameters
        self._control_rate = 10.0  # Hz
        self._pos_threshold = 0.05  # meters
        self._default_velocity = 0.3  # m/s
        
        # Setup action client
        self._client = actionlib.SimpleActionClient(
            action_ns,
            FollowJointTrajectoryAction
        )
        
        if not self._client.wait_for_server(rospy.Duration(5.0)):
            raise RuntimeError("Arm action server not available")
        
        self._wrist_force = None
        self._wrist_sub = rospy.Subscriber(
            '/hsrb/wrist_wrench/compensated',
            WrenchStamped,
            self._wrist_callback,
            queue_size=1
        )

    def __del__(self):
        """Cleanup resources"""
        if not rospy.is_shutdown():
            if hasattr(self, '_client'):
                self._client.cancel_all_goals()
            if hasattr(self, '_wrist_sub'):
                self._wrist_sub.unregister()

    def _wrist_callback(self, msg: WrenchStamped) -> None:
        """Store latest force readings from wrist sensor"""
        self._wrist_force = np.array([
            msg.wrench.force.x,
            msg.wrench.force.y,
            msg.wrench.force.z
        ])

    def set_joint_values(self, positions: List[float], 
                        velocities: List[float] = None,
                        timeout: float = 5.0) -> bool:
        """Send arm to specified joint positions"""
        try:
            if len(positions) != len(self._joint_names):
                raise ValueError(f"Expected {len(self._joint_names)} positions")
                
            # Create trajectory goal
            goal = FollowJointTrajectoryGoal()
            goal.trajectory.joint_names = self._joint_names
            
            point = JointTrajectoryPoint()
            point.positions = positions
            point.velocities = velocities or [0.1] * len(positions)
            point.time_from_start = rospy.Duration(0.5)
            goal.trajectory.points = [point]
            
            # Execute movement
            self._client.send_goal(goal)
            return self._client.wait_for_result(rospy.Duration(timeout))
            
        except Exception as e:
            rospy.logerr(f"Failed to set joint values: {e}")
            return False

    def get_joint_values(self) -> List[float]:
        """Get current joint positions"""
        try:
            states = rospy.wait_for_message('/hsrb/joint_states', JointState, timeout=1.0)
            return [states.position[1], states.position[0], 
                   states.position[2], states.position[11], states.position[12]]
        except Exception as e:
            rospy.logerr(f"Failed to get joint values: {e}")
            return []

    def set_named_target(self, pose: str = 'go') -> bool:
        """Move arm to predefined pose"""
        if pose not in self._named_poses:
            rospy.logwarn(f"Unknown pose '{pose}', using 'go' pose")
            pose = 'go'
        return self.set_joint_values(self._named_poses[pose])

    def check_grasp(self, 
                   weight_threshold: float = 0.5,
                   timeout: float = 1.0,
                   average_samples: int = 5) -> bool:
        """
        Check if object is grasped by measuring wrist sensor force
        
        Args:
            weight_threshold: Minimum force magnitude to consider successful grasp (N)
            timeout: Maximum time to wait for sensor readings
            average_samples: Number of samples to average for stable reading
            
        Returns:
            bool: True if object is detected in gripper
        """
        try:
            # Wait for sensor data
            start_time = rospy.Time.now()
            samples = []
            
            while len(samples) < average_samples and not rospy.is_shutdown():
                if self._wrist_force is not None:
                    # Get force magnitude (ignoring gravity compensation)
                    force_magnitude = np.linalg.norm(self._wrist_force)
                    samples.append(force_magnitude)
                
                # Check timeout
                if (rospy.Time.now() - start_time).to_sec() > timeout:
                    rospy.logwarn("Timeout waiting for force readings")
                    return False
                    
                rospy.sleep(0.05)  # 20Hz sampling

            if not samples:
                return False

            # Calculate average force
            avg_force = sum(samples) / len(samples)
            rospy.logdebug(f"Average force magnitude: {avg_force:.3f} N")
            
            # Check if force exceeds threshold
            return avg_force > weight_threshold

        except Exception as e:
            rospy.logerr(f"Grasp check failed: {e}")
            return False

    def move_hand_to_target(self, 
                          target_frame: str,
                          offset: List[float] = None,
                          threshold: float = 0.05,
                          max_velocity: float = 0.3,
                          timeout: float = 10.0) -> bool:
        """
        Move hand to target frame with smooth base movement
        
        Args:
            target_frame: Target TF frame
            offset: Optional [x,y,z] offset from target
            threshold: Position error threshold
            max_velocity: Maximum base velocity
            timeout: Movement timeout
        """
        offset = offset or [0, 0, 0]
        hand_frame = 'hand_palm_link'
        base_frame = 'base_link'
        rate = rospy.Rate(self._control_rate)
        start_time = rospy.Time.now()

        try:
            while not rospy.is_shutdown():
                # Check timeout
                if (rospy.Time.now() - start_time).to_sec() > timeout:
                    rospy.logwarn("Hand movement timed out")
                    return False

                # Get transforms
                hand_pos, _ = self._tf_man.get_transform(
                    target_frame = hand_frame, 
                    ref_frame = base_frame)
                
                target_pos, _ = self._tf_man.get_transform(
                    target_frame = target_frame, 
                    ref_frame = base_frame)

                if not hand_pos or not target_pos:
                    continue

                # Calculate error
                error = np.array(target_pos) - np.array(hand_pos) - np.array(offset)
                error[2] = 0  # Ignore Z-axis
                error_norm = np.linalg.norm(error[:2])

                # Check if target reached
                if error_norm < threshold:
                    return True

                # Calculate velocities (proportional control)
                vel_x = 0.4 * error[0]
                vel_y = 0.6 * error[1]

                # Move base
                self._base.tiny_move(
                    x = vel_x,
                    y = vel_y,
                    duration = 0.2
                    # MAX_VEL=max_velocity
                )

                rate.sleep()

        except Exception as e:
            rospy.logerr(f"Hand movement failed: {e}")
            return False

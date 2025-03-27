import rospy
import numpy as np
from geometry_msgs.msg import Point, Quaternion 
from typing import List, Tuple, Optional
from geometry_msgs.msg import TransformStamped, Point, Quaternion
import tf2_ros as tf2
from tf import transformations


# ROS functionalities

class TFManager:
    """A class to manage ROS TF2 transforms with improved functionality."""
    
    def __init__(self):
        """Initialize TF manager with required publishers and listeners."""
        self._tf_buffer = tf2.Buffer()
        self._tf_listener = tf2.TransformListener(self._tf_buffer)
        self._static_broadcaster = tf2.StaticTransformBroadcaster()
        self._broadcaster = tf2.TransformBroadcaster()
        
    def create_transform(self, 
                        position: List[float],
                        rotation: List[float],
                        child_frame: str,
                        parent_frame: str = "/map",
                        stamp: Optional[rospy.Time] = None) -> TransformStamped:
        """
        Create a TransformStamped message.
        
        Args:
            position: [x, y, z] coordinates
            rotation: [x, y, z, w] quaternion or [roll, pitch, yaw] euler angles
            child_frame: Name of the child frame
            parent_frame: Name of the parent frame
            stamp: ROS timestamp (defaults to current time)
            
        Returns:
            TransformStamped message
        """
        if not stamp:
            stamp = rospy.Time.now()
            
        # Convert euler angles to quaternion if needed
        if len(rotation) == 3:
            rotation = transformations.quaternion_from_euler(*rotation)
            
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = parent_frame
        transform.child_frame_id = child_frame
        transform.transform.translation = Point(*position)
        transform.transform.rotation = Quaternion(*rotation)
        
        return transform
    
    def publish_transform(self,
                         position: List[float],
                         rotation: List[float],
                         child_frame: str,
                         parent_frame: str = "map",
                         static: bool = False) -> None:
        """
        Publish a transform either as static or dynamic.
        
        Args:
            position: [x, y, z] coordinates
            rotation: [x, y, z, w] quaternion or [roll, pitch, yaw] euler angles
            child_frame: Name of the child frame
            parent_frame: Name of the parent frame
            static: If True, publishes as static transform
        """
        transform = self.create_transform(position, rotation, child_frame, parent_frame)
        
        if static:
            self._static_broadcaster.sendTransform(transform)
        else:
            self._broadcaster.sendTransform(transform)
            
    def get_transform(self,
                     target_frame: str,
                     source_frame: str,
                     timeout: float = 1.0) -> Tuple[Optional[List[float]], Optional[List[float]]]:
        """
        Get the transform between two frames.
        
        Args:
            target_frame: Name of the target frame
            source_frame: Name of the source frame
            timeout: Time to wait for transform in seconds
            
        Returns:
            Tuple of ([x, y, z], [x, y, z, w]) or (None, None) if transform not found
        """
        try:
            transform = self._tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rospy.Time(0),
                rospy.Duration(timeout)
            )
            
            position = [
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z
            ]
            
            rotation = [
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            ]
            
            return position, rotation
            
        except (tf2.LookupException, tf2.ConnectivityException, 
                tf2.ExtrapolationException) as e:
            rospy.logwarn(f"Failed to lookup transform: {e}")
            return None, None
                
    def transform_pose(self,
                    position: List[float],
                    rotation: List[float], 
                    from_frame: str,
                    to_frame: str) -> Tuple[Optional[List[float]], Optional[List[float]]]:
        """
        Transform a pose from one frame to another using homogeneous transformations.
        
        Args:
            position: [x, y, z] coordinates in source frame
            rotation: [x, y, z, w] quaternion in source frame
            from_frame: Source frame
            to_frame: Target frame
            
        Returns:
            Transformed position and rotation or (None, None) if transform fails
        """
        try:
            transform = self._tf_buffer.lookup_transform(
                to_frame,
                from_frame,
                rospy.Time(0)
            )
            
            # Create 4x4 transformation matrices
            def create_transform_matrix(pos, quat):
                # Convert quaternion to rotation matrix
                rot_matrix = transformations.quaternion_matrix(quat)
                # Set translation
                rot_matrix[0:3, 3] = pos
                return rot_matrix
                
            # Create transform matrices
            tf_matrix = create_transform_matrix(
                [transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z],
                [transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w]
            )
            
            pose_matrix = create_transform_matrix(position, rotation)
            
            # Transform the pose
            result_matrix = np.dot(tf_matrix, pose_matrix)
            
            # Extract position
            transformed_position = result_matrix[0:3, 3].tolist()
            
            # Extract rotation and convert to quaternion
            transformed_rotation = transformations.quaternion_from_matrix(result_matrix)
            
            return transformed_position, transformed_rotation.tolist()
            
        except (tf2.LookupException, tf2.ConnectivityException,
                tf2.ExtrapolationException) as e:
            rospy.logwarn(f"Failed to transform pose: {e}")
            return None, None
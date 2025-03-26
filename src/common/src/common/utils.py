import rospy
import yaml
import rospkg
import tf
import numpy as np
from geometry_msgs.msg import Twist, Point, Quaternion, PoseStamped
import actionlib
from actionlib_msgs.msg import GoalStatus
# from tmc_msgs.msg import TalkRequestActionGoal, TalkRequestAction
from tmc_msgs.msg import Voice 
from typing import Tuple

class Talker:
    def __init__(self, talk_request_topic='/talk_request'):
        """Initialize the talker with ROS publisher"""
        try:
            self.talk_pub = rospy.Publisher(talk_request_topic, Voice, queue_size=10)
            rospy.loginfo("Waiting for publisher to connect...")
            rospy.sleep(1.0)  # Esperamos un momento para asegurarnos de que se haya conectado
            rospy.loginfo(f"Connected to {talk_request_topic}!")
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

    def talk(self, sentence: str, timeout: float = 5.0) -> bool:
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
            rospy.sleep(timeout)  # Esperamos por el tiempo que definimos
            
            return True  # Suponemos que el mensaje fue publicado correctamente
            
        except Exception as e:
            rospy.logwarn(f"Failed to send talk message: {e}")
            return False

class VelocityMovement:
    """Handles direct velocity commands for the robot"""
    def __init__(self, publisher):
        self.vel_pub = publisher
        self.vel = Twist()
    
    def set_velocity(self, x: float, y: float, theta: float):
        """Set linear and angular velocities"""
        self.vel.linear.x = x
        self.vel.linear.y = y 
        self.vel.angular.z = theta

    def move(self, duration: float = 0.5):
        """Execute movement for specified duration"""
        start_time = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - start_time < duration:
            self.vel_pub.publish(self.vel)
            rospy.sleep(0.1)

class LocationManager:
    """Manages known locations from YAML file"""
    def __init__(self, file_path: str = '/known_locations.yaml'):
        self.file_path = file_path
        self._locations = self._load_locations()
    
    def _load_locations(self) -> dict:
        """Load locations from YAML file"""
        try:
            rospack = rospkg.RosPack()
            full_path = rospack.get_path('config_files') + self.file_path
            with open(full_path, 'r') as file:
                return yaml.safe_load(file)
        except Exception as e:
            rospy.logerr(f"Failed to load locations: {e}")
            return {}
    
    def get_location(self, name: str) -> Tuple[float, float, float]:
        """Get x, y, theta for a known location"""
        try:
            loc = self._locations[name][:3]
            return (loc[0]['x'], loc[1]['y'], loc[2]['theta'])
        except (KeyError, IndexError):
            rospy.logerr(f"Location '{name}' not found")
            return None

class Navigation:
    """Handles robot navigation combining velocity control and move_base"""
    def __init__(self, 
                 cmd_vel_topic: str = '/hsrb/command_velocity',
                 locations_file: str = '/known_locations.yaml'):
        
        # Publishers and Subscribers
        self._vel_pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
        self._nav_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self._status_sub = rospy.Subscriber('/navigation/status', GoalStatus, self._nav_status_cb)
        
        # Movement handlers
        self._velocity = VelocityMovement(self._vel_pub)
        self._locations = LocationManager(locations_file)
        
        # Navigation status
        self.arrived = False
        self.failed = False
        
        # Movement constraints
        self.MAX_VEL_LINEAR = 0.03
        self.MAX_VEL_ANGULAR = 0.5

    def _nav_status_cb(self, msg: GoalStatus):
        """Callback for navigation status"""
        rospy.loginfo(f"Navigation status: {msg.status}")
        self.arrived = msg.status == 3  # SUCCEEDED
        self.failed = msg.status == 4   # ABORTED

    def tiny_move(self, x: float = 0, y: float = 0, theta: float = 0, 
                 duration: float = 0.5):
        """Execute small movement using velocity commands"""
        # Limit velocities
        x = np.clip(x, -self.MAX_VEL_LINEAR, self.MAX_VEL_LINEAR)
        y = np.clip(y, -self.MAX_VEL_LINEAR, self.MAX_VEL_LINEAR)
        theta = np.clip(theta, -self.MAX_VEL_ANGULAR, self.MAX_VEL_ANGULAR)
        
        self._velocity.set_velocity(x, y, theta)
        self._velocity.move(duration)

    def move_to(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0, 
                location: str = None, timeout: float = 90.0) -> bool:
        """Navigate to goal position or known location"""
        # Reset status flags
        self.arrived = False
        self.failed = False
        
        # Get coordinates
        if location:
            coords = self._locations.get_location(location)
            if not coords:
                return False
            x, y, theta = coords
            
        # Create and publish goal
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = rospy.Time.now()
        goal.pose.position = Point(x, y, 0)
        goal.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, theta))
        
        self._nav_pub.publish(goal)
        
        # Wait for result
        rate = rospy.Rate(10)
        start_time = rospy.Time.now()
        
        while not rospy.is_shutdown():
            if self.arrived:
                return True
            if self.failed:
                return False
            if (rospy.Time.now() - start_time).to_sec() > timeout:
                rospy.logwarn("Navigation timeout")
                return False
            rate.sleep()
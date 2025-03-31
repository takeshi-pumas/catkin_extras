#!/usr/bin/env python3

import rospy
import smach
from typing import Callable, Optional
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import LaserScan
from common.hsr_functions import Talker
from common.navigation_functions import Navigation

class WaitPushHand(smach.State):
    """Common state for waiting for the hand push interaction"""
    def __init__(self,
                 talker: Optional[Talker] = None,
                 talk_message: str = 'Gently, push my hand to begin.',
                 timeout: float = 100.0, 
                 push_threshold: float = 1.0):
        
        """
        Initialize WaitPushHand state
        
        Args:
            talker: Optional object for speaking
            talk_message: Message to speak when waiting for push
            timeout: Maximum time to wait for push in seconds
            push_threshold: Force threshold to detect push
        """
        smach.State.__init__(self, outcomes=['succ', 'failed', 'aborted'])

        self.push_threshold = push_threshold
        self.timeout = timeout
        self.talker = talker
        self.talk_message = talk_message
        self.counter = 0

        topic_wrench_sensor = '/hsrb/wrist_wrench/compensated'
        self.wrist_sensor_sub = rospy.Subscriber(
            topic_wrench_sensor, 
            WrenchStamped, 
            self.wrist_callback)
        self.pushed = False
         
    def wrist_callback(self, msg):
        """Callback function for wrist sensor"""
        self.pushed = msg.wrench.force.x > self.push_threshold

    def execute(self, userdata) -> str:
        """Excecute the state"""
        print("State: Wait for push hand")

        if self.talker:
            try:
                self.talker.talk(self.talk_message)
            except Exception as e:
                print(f"Error talking: {e}")

        start_time = rospy.get_time()

        while not rospy.is_shutdown():
            self.counter += 1
            if self.pushed:
                print('Hand push detected')
                self.wrist_sensor_sub.unregister()
                return 'succ'

            if self.counter % 20 == 0:
                print(f'Waiting for hand push ({self.counter // 20})')
            
            if rospy.get_time() - start_time > self.timeout:
                print('Timeout waiting for push')
                return 'failed'
            
            rospy.sleep(0.1)
        
        return 'aborted'
    
    def __del__(self):
        """Cleanup subscriber on deletion"""
        if hasattr(self, 'wrist_sensor_sub'):
            self.wrist_sensor_sub.unregister()

class WaitDoorOpen(smach.State):
    """Common state for waiting for the door to be opened"""
    def __init__(self,
                 talker: Optional[Talker] = None,
                 talk_message: str = 'Please open the door.',
                 timeout: float = 100.0,
                 distance_threshold: float = 0.5):
        
        """
        Initialize WaitDoorOpen state
        
        Args:
            talker: Optional object for speaking
            talk_message: Message to speak when waiting for door
            timeout: Maximum time to wait for door to open in seconds
        """
        smach.State.__init__(self, outcomes=['succ', 'failed', 'aborted'])

        self.timeout = timeout
        self.talker = talker
        self.talk_message = talk_message
        self.counter = 0

        topic_laser_scan = '/hsrb/base_scan'
        self.laser_sensor_sub = rospy.Subscriber(
            topic_laser_scan, 
            LaserScan, 
            self.laser_callback)
        self.door_opened = False
         
    def laser_callback(self, msg):
        """Callback function for door sensor"""
        # Obtener las lecturas del láser
        ranges = msg.ranges

        # Tomar solo las muestras centrales
        num_samples = len(ranges)
        num_central_samples = int(num_samples * 0.05)
        start_index = (num_samples - num_central_samples) // 2
        central_ranges = ranges[start_index: start_index + num_central_samples]

        # Calcular la media de las lecturas
        mean = sum(central_ranges) / len(central_ranges)

        self.door_opened = mean > 0.6

    def execute(self, userdata) -> str:
        """Excecute the state"""
        print("State: Wait for door open")

        if self.talker:
            try:
                self.talker.talk(self.talk_message)
            except Exception as e:
                print(f"Error talking: {e}")

        start_time = rospy.get_time()

        while not rospy.is_shutdown():
            self.counter += 1
            if self.counter % 20 == 0:
                print(f'Waiting for door to open ({self.counter // 20})')

            if self.door_opened:
                print('Door opened')
                self.laser_sensor_sub.unregister()
                return 'succ'

            if rospy.get_time() - start_time > self.timeout:
                print('Timeout waiting for door')
                return 'failed'
            
            rospy.sleep(0.1)
        
        return 'aborted'
    
    def __del__(self):
        """Cleanup subscriber on deletion"""
        if hasattr(self, 'laser_sensor_sub'):
            self.laser_sensor_sub.unregister()

class GotoPlace(smach.State):
    """Common state for navigating to a specific place"""
    def __init__(self,
                 navigation: Navigation,
                 x: Optional[float] = None,
                 y: Optional[float] = None,
                 theta: Optional[float] = None,
                 location: Optional[str] = None,
                 talker: Optional[Talker] = None,
                 start_message: Optional[str] = None,
                 end_message: Optional[str] = None,
                 timeout: float = 90.0,
                 debug: bool = False):
        """
        Initialize GotoPlace state
        
        Args:
            navigation: Navigation instance for movement
            x: X coordinate (ignored if location is provided)
            y: Y coordinate (ignored if location is provided)
            theta: Orientation in radians (ignored if location is provided)
            location: Name of known location (takes precedence over x,y,theta)
            talker: Optional Voice instance for speaking
            start_message: Message to speak when starting navigation
            end_message: Message to speak when navigation completes
            timeout: Maximum time to wait for navigation
            debug: Dontify if debug mode is enabled
        """
        smach.State.__init__(self, outcomes=['succ', 'failed', 'aborted'])
        
        if location is None and (x is None or y is None or theta is None):
            raise ValueError("Must provide either location name or x,y,theta coordinates")
            
        self.navigation = navigation
        self.x = x
        self.y = y
        self.theta = theta
        self.location = location
        self.talker = talker
        self.start_message = start_message
        self.end_message = end_message
        self.timeout = timeout
        self.debug = debug

    def execute(self, userdata) -> str:
        """Execute the navigation state"""
        rospy.loginfo("State: Go to place")

        # Announce start if talker available
        if self.talker and self.start_message:
            try:
                self.talker.talk(self.start_message)
            except Exception as e:
                rospy.logwarn(f"Failed to announce start: {e}")
        
        if self.debug:
            if self.talker and self.end_message:
                    try:
                        self.talker.talk(self.end_message)
                    except Exception as e:
                        rospy.logwarn(f"Failed to announce completion: {e}")
            return 'succ'

        try:
            # Navigate using either location name or coordinates
            if self.location:
                success = self.navigation.move_to(known_location=self.location, 
                                                  timeout=self.timeout)
            else:
                success = self.navigation.move_to(goal_x=self.x,
                                                  goal_y=self.y, 
                                                  goal_theta=self.theta,
                                                  timeout=self.timeout)
            
            if success:
                # Announce completion if talker available
                if self.talker and self.end_message:
                    try:
                        self.talker.talk(self.end_message)
                    except Exception as e:
                        rospy.logwarn(f"Failed to announce completion: {e}")
                return 'succ'
            else:
                return 'failed'
                
        except rospy.ROSInterruptException:
            # rospy.logerr(f"Navigation failed: {e}")
            return 'aborted'
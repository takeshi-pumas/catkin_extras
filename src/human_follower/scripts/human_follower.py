#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PointStamped, Twist
import tf2_ros
import tf2_geometry_msgs
import math
from std_msgs.msg import Bool, Empty

class HumanFollower:
    def __init__(self):
        self.current_target = None
        self.is_last_waypoint = False
        #self.current_position = Point(0, 0, 0)  # Asumimos que el robot empieza en el origen

        # Suscribirse a los waypoints
        self.subscriber = rospy.Subscriber('/hri/active_waypoint', PointStamped, self.target_callback)

         # Suscribirse al estado de si estamos en el último waypoint
        self.last_waypoint_subscriber = rospy.Subscriber('/hri/last_waypoint', Bool, self.last_waypoint_callback)

        # Publicar comandos de movimiento
        self.cmd_vel_publisher = rospy.Publisher("/hsrb/command_velocity", Twist, queue_size=10)

        # Publicar para pedir el siguiente elemento en la lista
        self.next_element_publisher = rospy.Publisher('/hri/request_next_waypoint', Empty, queue_size=10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.rate = rospy.Rate(30)  # 30 Hz

    def target_callback(self, msg):
        self.current_target = msg

    def last_waypoint_callback(self, msg):
        self.is_last_waypoint = msg.data

    def get_robot_position(self):
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rospy.Time(0), rospy.Duration(1.0))
            return tf2_geometry_msgs.do_transform_point(PointStamped(header=transform.header), transform)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Failed to get robot position")
            return None
        

    def move_to_target(self):
        while not rospy.is_shutdown():
            if self.current_target is not None:
                current_position = self.get_robot_position()

                if current_position is not None:

                    print(f"current position: {current_position.point}")
                    print(f"target_position: {self.current_target.point}")
                    distance = self.calculate_distance(current_position.point, self.current_target.point)
                    threshold = 1.0 if self.is_last_waypoint else 0.1
                    if distance > threshold:  # Si la distancia es mayor al umbral
                        cmd_vel = self.calculate_velocity(current_position.point, self.current_target.point)
                        self.cmd_vel_publisher.publish(cmd_vel)
                    else:
                        # Si hemos llegado al objetivo, detenemos el robot
                        self.cmd_vel_publisher.publish(Twist())
                        # Pedir el siguiente elemento en la lista
                        self.next_element_publisher.publish(Empty())
            self.rate.sleep()

    def calculate_distance(self, pos1, pos2):
        return math.sqrt((pos1.x - pos2.x)**2 + (pos1.y - pos2.y)**2)

    def calculate_velocity(self, current_position, target_position):
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.4 * (target_position.x - current_position.x)
        cmd_vel.linear.y = 0.4 * (target_position.y - current_position.y)
        return cmd_vel

if __name__ == '__main__':
    rospy.init_node('human_follower_node')
    mover = HumanFollower()
    mover.move_to_target()

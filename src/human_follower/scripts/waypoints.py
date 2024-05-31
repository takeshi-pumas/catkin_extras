#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Bool, Empty
from geometry_msgs.msg import PointStamped
import math
import tf2_ros
import tf2_geometry_msgs

class WaypointsNode:
    def __init__(self):
        self.waypoints = []
        self.current_index = 0
        leg_pose_topic = "/hri/leg_finder/leg_pose"
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        print("Waypoints node active")

        # Suscribirse al tópico de entrada
        print(f"Waiting for {leg_pose_topic}")
        rospy.wait_for_message(leg_pose_topic, PointStamped, timeout=2.0)
        self.subscriber = rospy.Subscriber("/hri/leg_finder/leg_pose", PointStamped, self.new_legs_callback)

        # Publicar el elemento actual
        self.publisher = rospy.Publisher('/hri/active_waypoint', PointStamped, queue_size=10)
        self.end_of_list_publisher = rospy.Publisher('/hri/last_waypoint', Bool, queue_size=10)

        # Suscribirse al tópico para cambiar el elemento actual
        self.change_index_subscriber = rospy.Subscriber('/hri/request_next_waypoint', Empty, self.change_index_callback)

        self.timer = rospy.Timer(rospy.Duration(2), self.add_waypoint)

    def transform_point_to_map(self, point_stamped):
        try:
            transform = self.tf_buffer.lookup_transform('map', point_stamped.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
            transformed_point = tf2_geometry_msgs.do_transform_point(point_stamped, transform)
            return transformed_point
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Failed to transform point from %s to map", point_stamped.header.frame_id)
            return None

    def new_legs_callback(self, msg):

        self.current_position = self.transform_point_to_map(msg)
        if self.waypoints:
            self.publisher.publish(self.waypoints[self.current_index])
            


    def is_far_enough(self, last_point, new_point):
        distance = math.sqrt((last_point.x - new_point.x)**2 + (last_point.y - new_point.y)**2)
        return distance >= 0.25

    def add_waypoint(self, event):
        if not self.waypoints or self.is_far_enough(self.waypoints[-1].point, self.current_position.point):
            print(f"New waypoint added: {self.current_index} / {len(self.waypoints)}")
            self.waypoints.append(self.current_position)
            
        

    def change_index_callback(self, msg):
        is_last = self.current_index == len(self.waypoints) - 1
        if not is_last:
            self.current_index += 1
        self.end_of_list_publisher.publish(Bool(data=is_last))
        
            

if __name__ == '__main__':
    rospy.init_node('waypoints_node')
    node = WaypointsNode()
    rospy.spin()
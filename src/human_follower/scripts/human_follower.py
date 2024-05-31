#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PointStamped, Twist, Quaternion
import tf2_ros
import tf2_geometry_msgs
import math
from std_msgs.msg import Bool, Empty
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan

class HumanFollower:
    def __init__(self):
        self.current_target = None
        #self.is_last_waypoint = True
        
        self.dist_to_target = 1.0
        self.control_alpha = 0.6584
        self.control_beta = 0.3
        self.max_linear = 0.25
        self.max_angular = 0.7
        self.kInfRep = 0.05
        self.Fx_rep = 0.0
        self.Fy_rep = 0.0

        print("Waiting for Laser Message")
        msg = rospy.wait_for_message("/hsrb/base_scan", LaserScan)
        self.laser_low_limit = int(2/5 * len(msg.ranges))
        self.laser_high_limit = int(3/5 * len(msg.ranges))

        self.sub_active_waypoint_sub = rospy.Subscriber('/hri/active_waypoint', PointStamped, self.target_callback)
        self.sub_last_waypoint = rospy.Subscriber('/hri/last_waypoint', Bool, self.last_waypoint_callback)
        self.sub_lidar = rospy.Subscriber("/hsrb/base_scan", LaserScan, self.callback_lidar)
        self.pub_cmd_vel = rospy.Publisher("/hsrb/command_velocity", Twist, queue_size=10)
        self.pub_next_element = rospy.Publisher('/hri/request_next_waypoint', Empty, queue_size=10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.rate = rospy.Rate(30)  # 30 Hz

    def target_callback(self, msg):
        self.current_target = msg

    def last_waypoint_callback(self, msg):
        self.dist_to_target = 1.0 if msg.data else 0.40
        #self.is_last_waypoint = msg.data

    def callback_lidar(self, msg):
        self.Fx_rep = 0.0
        self.Fy_rep = 0.0
        laser_threshold = 0.95
        lectures = msg.ranges

        for i in range(self.laser_low_limit, self.laser_high_limit):
            if lectures[i] < laser_threshold:
                angle = msg.angle_min + i * msg.angle_increment
                force = 1 / lectures[i]**2
                self.Fx_rep = force * math.cos(angle)
                self.Fy_rep = force * math.sin(angle)
        

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
                    if distance > self.dist_to_target:  # Si la distancia es mayor al umbral
                        cmd_vel = self.calculate_velocity(current_position, self.current_target)
                        self.pub_cmd_vel.publish(cmd_vel)
                    else:
                        # Si hemos llegado al objetivo, detenemos el robot
                        self.pub_cmd_vel.publish(Twist())
                        # Pedir el siguiente elemento en la lista
                        self.pub_next_element.publish(Empty())
            self.rate.sleep()

    def calculate_distance(self, pos1, pos2):
        return math.sqrt((pos1.x - pos2.x)**2 + (pos1.y - pos2.y)**2)

    def calculate_velocity(self, current_position, target_position):
        try:
            transform = self.tf_buffer.lookup_transform("base_link", "map", rospy.Time(0), rospy.Duration(1.0))
            transformed_target = tf2_geometry_msgs.do_transform_point(target_position, transform).point

            deltax = transformed_target.x
            deltay = transformed_target.y

            angle_error = math.atan2(deltay, deltax)
            distance = math.sqrt(deltax**2 + deltay**2) - self.dist_to_target

            distance = max(distance, self.max_linear)

            cmd_vel = Twist()
            if distance > 0 :
                
                cmd_vel.linear.x = distance * math.exp(-(angle_error**2) / self.control_alpha) - self.kInfRep * self.Fx_rep
                cmd_vel.linear.y = 0 - self.kInfRep * self.Fy_rep
                cmd_vel.angular.z = self.max_angular * (2 / (1 + math.exp(-angle_error / self.control_beta)) -1)
            else:
                cmd_vel.linear.x = 0
                cmd_vel.linear.y = 0
                if abs(angle_error) >= math.pi / 24:
                    cmd_vel.angular.z = self.max_angular * (2 / (1 + math.exp(-angle_error / self.control_beta)) -1)
                else:
                    cmd_vel.angular.z = 0
        
            return cmd_vel
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Failed to transform target position to base_link frame")
            return Twist()

if __name__ == '__main__':
    rospy.init_node('human_follower_node')
    mover = HumanFollower()
    mover.move_to_target()

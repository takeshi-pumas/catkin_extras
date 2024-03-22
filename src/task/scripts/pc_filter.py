#!/usr/bin/env python

import rospy
import tf2_ros
import tf2_geometry_msgs
import numpy as np
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TransformStamped
from sensor_msgs import point_cloud2
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from std_msgs.msg import Float32

class PointCloudTransformer:
    def __init__(self):
        rospy.init_node('point_cloud_transformer_node')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.input_topic = "/hsrb/head_rgbd_sensor/depth_registered/rectified_points"
        self.output_topic = "/point_cloud_transformed"
        self.filter_threshold = 0.01
        self.sub = rospy.Subscriber(self.input_topic, PointCloud2, self.point_cloud_callback)
        self.sub_threshold = rospy.Subscriber("/pc/change_threshold", Float32, self.threshold_callback)
        self.pub = rospy.Publisher(self.output_topic, PointCloud2, queue_size=1)
        
    def threshold_callback(self, msg:Float32):
        self.filter_threshold = msg.data
        
    def point_cloud_callback(self, msg):
        try:
            transform = self.tf_buffer.lookup_transform("map", msg.header.frame_id, rospy.Time(0), rospy.Duration(1))
            transformed_cloud = do_transform_cloud(msg, transform)
            
            pcl_cloud = point_cloud2.read_points(transformed_cloud, field_names=("x", "y", "z"))
            filtered_pcl_cloud = []

            for point in pcl_cloud:
                if point[2] >= self.filter_threshold:
                    filtered_pcl_cloud.append(point)

            header = transformed_cloud.header
            filtered_cloud = point_cloud2.create_cloud_xyz32(header, filtered_pcl_cloud)

            self.pub.publish(filtered_cloud)
        except Exception as e:
            rospy.logwarn("Transform exception: %s", e)

if __name__ == '__main__':
    try:
        pc_transformer = PointCloudTransformer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

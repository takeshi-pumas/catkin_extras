/// @brief Plane Detection Sample
/// @brief Copyright (C) 2016 Toyota Motor Corporation
#include <string>
#include <hsrb_vision_samples/plane_detection.hpp>

namespace hsrb_vision_samples {

PlaneDetection::PlaneDetection()
  : output_cloud_(new pcl::PointCloud<pcl::PointXYZRGB>) {
  // subscribe point cloud data from HSR
  std::string topic_name = "/hsrb/head_rgbd_sensor/depth_registered/rectified_points";
  cloud_sub_ = nh_.subscribe(topic_name, 1, &PlaneDetection::CloudCb, this);

  if (!ros::topic::waitForMessage<sensor_msgs::PointCloud2>(topic_name, ros::Duration(10.0))) {
    ROS_ERROR("timeout exceeded while waiting for message on topic %s", topic_name.c_str());
    exit(EXIT_FAILURE);
  }

  // publish output cloud
  plane_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/plane_detection_output", 1);
}

pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr PlaneDetection::GetOutputCloud() const {
  return PlaneDetection::output_cloud_;
}

void PlaneDetection::CloudCb(const sensor_msgs::PointCloud2ConstPtr& msg) {
  // convert message type
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  pcl::fromROSMsg(*msg, cloud);

  // plane model segmentation
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.01);
  seg.setInputCloud(cloud.makeShared());

  pcl::PointIndices inliers;
  pcl::ModelCoefficients coefficients;
  seg.segment(inliers, coefficients);

  // paint a plane red
  pcl::copyPointCloud(cloud, *output_cloud_);
  for (size_t i = 0; i < inliers.indices.size(); i++) {
    output_cloud_->points[inliers.indices[i]].r = 255;
    output_cloud_->points[inliers.indices[i]].g = 0;
    output_cloud_->points[inliers.indices[i]].b = 0;
  }

  // publish detected plane cloud
  sensor_msgs::PointCloud2 pub_cloud;
  pcl::toROSMsg(*output_cloud_, pub_cloud);
  plane_pub_.publish(pub_cloud);
}
}  // namespace hsrb_vision_samples

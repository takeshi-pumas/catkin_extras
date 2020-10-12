/// @brief Plane Detection Sample
/// @brief Copyright (C) 2016 Toyota Motor Corporation

#ifndef HSRB_VISION_SAMPLES_PLANE_DETECTION_HPP_
#define HSRB_VISION_SAMPLES_PLANE_DETECTION_HPP_

#include <string>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_msgs/ModelCoefficients.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>


namespace hsrb_vision_samples {
/// @class PlaneDetection
/// @brief detecting and painting a plane class using Point Cloud Library
class PlaneDetection {
 public:
  PlaneDetection();
  ~PlaneDetection() {}
  pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr GetOutputCloud() const;

 private:
  ros::NodeHandle nh_;
  ros::Subscriber cloud_sub_;
  ros::Publisher plane_pub_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud_;

  void CloudCb(const sensor_msgs::PointCloud2ConstPtr& msg);
};
}  // namespace hsrb_vision_samples

#endif  // HSRB_VISION_SAMPLES_COLOR_DETECTION_HPP_

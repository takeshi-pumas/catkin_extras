/// @brief Flesh Color Detection Sample
/// @brief Copyright (C) 2016 Toyota Motor Corporation

#ifndef HSRB_VISION_SAMPLES_COLOR_DETECTION_HPP_
#define HSRB_VISION_SAMPLES_COLOR_DETECTION_HPP_

#include <string>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

namespace hsrb_vision_samples {
/// @class ColorDetection
/// @brief color detection class using OpenCV
class ColorDetection {
 public:
  ColorDetection();
  virtual ~ColorDetection() {}
  void ExtractColor(cv::Mat& dst_image) const;

 private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  cv::Mat input_image_;
  void ColorImageCb(const sensor_msgs::ImageConstPtr& msg);
};
}  // namespace hsrb_vision_samples

#endif  // HSRB_VISION_SAMPLES_COLOR_DETECTION_HPP_

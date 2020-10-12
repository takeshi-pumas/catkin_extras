/// @brief Flesh Color Detection Sample
/// @brief Copyright (C) 2016 Toyota Motor Corporation
#include <string>
#include <hsrb_vision_samples/color_detection.hpp>

namespace {
const uint8_t kHLowThreshold = 0;
const uint8_t kHHighThreshold = 15;
const uint8_t kSLowThreshold = 50;
const uint8_t kSHighThreshold = 255;
const uint8_t kVLowThreshold = 50;
const uint8_t kVHighThreshold = 255;

void CreateMaskImage(const cv::Mat& src, uint8_t th_max, uint8_t th_min, cv::Mat& dst) {
  cv::Mat th_src;
  cv::threshold(src, th_src, th_max, 255, CV_THRESH_TOZERO_INV);
  cv::threshold(th_src, dst, th_min, 255, CV_THRESH_BINARY);
}
}  // unnamed namespace


namespace hsrb_vision_samples {

ColorDetection::ColorDetection() : it_(nh_) {
  std::string topic_name = "/hsrb/head_rgbd_sensor/rgb/image_rect_color";
  // Subscribe color image data from HSR
  image_sub_ = it_.subscribe(topic_name, 1, &ColorDetection::ColorImageCb, this);

  if (!ros::topic::waitForMessage<sensor_msgs::Image>(topic_name, ros::Duration(5.0))) {
    ROS_ERROR("timeout exceeded while waiting for message on topic %s", topic_name.c_str());
    exit(EXIT_FAILURE);
  }
}

void ColorDetection::ExtractColor(cv::Mat& dst_image) const {
  // BGR -> HSV
  cv::Mat hsv;
  cv::cvtColor(input_image_, hsv, CV_BGR2HSV);

  // Break down channels
  cv::Mat h;
  cv::extractChannel(hsv, h, 0);
  cv::Mat s;
  cv::extractChannel(hsv, s, 1);
  cv::Mat v;
  cv::extractChannel(hsv, v, 2);

  // Create mask images(H, S, V)
  cv::Mat h_mask;
  CreateMaskImage(h, kHHighThreshold, kHLowThreshold, h_mask);
  cv::Mat s_mask;
  CreateMaskImage(s, kSHighThreshold, kSLowThreshold, s_mask);
  cv::Mat v_mask;
  CreateMaskImage(v, kVHighThreshold, kVLowThreshold, v_mask);

  cv::bitwise_and(h_mask, s_mask, dst_image);
  cv::bitwise_and(dst_image, v_mask, dst_image);
}

void ColorDetection::ColorImageCb(const sensor_msgs::ImageConstPtr& msg) {
  try {
    cv_bridge::CvImagePtr cv_ptr =
      cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    input_image_ = cv_ptr->image;
  } catch (cv_bridge::Exception& cv_bridge_exception) {
    ROS_ERROR("%s", cv_bridge_exception.what());
    return;
  }
}
}  // namespace hsrb_vision_samples

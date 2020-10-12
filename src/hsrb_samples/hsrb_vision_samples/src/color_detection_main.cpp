/// @brief Flesh Color Detection Sample
/// @brief Copyright (C) 2016 Toyota Motor Corporation
#include <hsrb_vision_samples/color_detection.hpp>

namespace {
const char* const kImageWindow = "Color Detection Image Window";
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "color_detection");
  hsrb_vision_samples::ColorDetection color_detection;

  cv::Mat dst_image;
  ros::Rate spin_rate(30);

  cv::namedWindow(kImageWindow);

  // Update GUI Window
  while (ros::ok()) {
    ros::spinOnce();
    color_detection.ExtractColor(dst_image);
    cv::imshow(kImageWindow, dst_image);
    cv::waitKey(3);
    spin_rate.sleep();
  }

  cv::destroyWindow(kImageWindow);

  return 0;
}

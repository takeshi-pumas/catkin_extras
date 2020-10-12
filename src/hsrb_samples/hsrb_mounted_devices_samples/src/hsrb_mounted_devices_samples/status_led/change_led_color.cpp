/// @brief Change Status LED Color Sample
/// @brief Copyright (C) 2017 Toyota Motor Corporation
#include <string>

#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>

namespace {
const uint16_t kLEDInputDataSize = 256;
const float kDefaultColorG = 0.2;
const float kDefaultColorB = 0.6;
}  // unnamed namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "hsrb_change_led_color");

  // Create Publisher to change status led color
  ros::NodeHandle nh;
  std::string status_led_topic = "/hsrb/command_status_led_rgb";
  ros::Publisher led_pub =
      nh.advertise<std_msgs::ColorRGBA>(status_led_topic, 100);

  // Wait for connection
  while (led_pub.getNumSubscribers() == 0) {
    ros::Duration(0.1).sleep();
  }

  // 50Hz is enough frequency to do gradual color change
  ros::Rate rate(50);

  // Value g and b are fixed
  std_msgs::ColorRGBA color;
  color.g = kDefaultColorG;
  color.b = kDefaultColorB;

  // Value r changes gradually
  for (uint16_t num = 0; num < kLEDInputDataSize; num++) {
    color.r = num / static_cast<float>(kLEDInputDataSize - 1);
    led_pub.publish(color);
    rate.sleep();
  }
  return EXIT_SUCCESS;
}

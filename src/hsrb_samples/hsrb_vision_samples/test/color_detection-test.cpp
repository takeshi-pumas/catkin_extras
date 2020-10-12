/// @brief C++色抽出のテスト
/// @brief Copyright (C) 2016 Toyota Motor Corporation
#include <stdlib.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <gtest/gtest.h>
#include <hsrb_vision_samples/color_detection.hpp>


TEST(AbnormalTest, NoSubscribeTest) {
  EXPECT_DEATH(hsrb_vision_samples::ColorDetection(), "");
}

TEST(NormalTest, OutputPubTest) {
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<sensor_msgs::Image>(
      "/hsrb/head_rgbd_sensor/rgb/image_rect_color", 1, true);

  // create sample picture for test
  sensor_msgs::Image sample_img;
  sample_img.header.stamp = ros::Time::now();
  sample_img.height = 3;
  sample_img.width = 4;
  sample_img.encoding = "bgr8";
  sample_img.is_bigendian = 0;
  sample_img.step = 12;
  sample_img.data.resize(sample_img.step*sample_img.height, 0);
  pub.publish(sample_img);

  hsrb_vision_samples::ColorDetection color_detection;
  cv::Mat dst_img;
  // spin for subscriber
  ros::spinOnce();
  color_detection.ExtractColor(dst_img);
  EXPECT_EQ(dst_img.size().height*dst_img.size().width, sample_img.height*sample_img.width);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "color_detection_test");
  ros::start();
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

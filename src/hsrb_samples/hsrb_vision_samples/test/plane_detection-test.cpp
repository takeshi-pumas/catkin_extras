/// @brief C++平面検出のテスト
/// @brief Copyright (C) 2016 Toyota Motor Corporation
#include <stdlib.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <gtest/gtest.h>
#include <hsrb_vision_samples/plane_detection.hpp>


class OutputCloudTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    output_is_published_ = false;
    sub_ = nh_.subscribe("/plane_detection_output", 1,
                         &OutputCloudTest::Callback, this);
    pub = nh_.advertise<sensor_msgs::PointCloud2>
      ("/hsrb/head_rgbd_sensor/depth_registered/rectified_points", 1, true);
  }

  virtual void TearDown() {
    sub_.shutdown();
  }

  bool IsPublished() {
    return output_is_published_;
  }
  ros::Publisher pub;

 private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  bool output_is_published_;

  // output cloudのscriber
  void Callback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    output_is_published_ = true;
  }
};


TEST(AbnormalTest, NoSubscribeTest) {
  EXPECT_DEATH(hsrb_vision_samples::PlaneDetection(), "");
}

TEST_F(OutputCloudTest, IsPublishedTest) {
  /// create sample cloud for test
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  cloud->width  = 15;
  cloud->height = 1;
  cloud->points.resize(cloud->width * cloud->height);
  uint32_t seed(0);
  for (size_t i = 0; i < cloud->points.size(); ++i) {
    cloud->points[i].x = 1024*rand_r(&seed) / (RAND_MAX + 1.0f);
    cloud->points[i].y = 1024*rand_r(&seed) / (RAND_MAX + 1.0f);
    cloud->points[i].z = 1.0;
    cloud->points[i].r = 255;
    cloud->points[i].g = 255;
    cloud->points[i].b = 255;
  }
  sensor_msgs::PointCloud2 sample_cloud;
  pcl::toROSMsg(*cloud, sample_cloud);

  pub.publish(sample_cloud);

  hsrb_vision_samples::PlaneDetection plane_detection;
  // spin for plane_detection subscriber
  ros::spinOnce();
  // spin for output subscriber
  ros::spinOnce();

  ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/plane_detection_output",
                                                       ros::Duration(5.0));
  EXPECT_TRUE(IsPublished());
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "plane_detection_test");
  ros::start();
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

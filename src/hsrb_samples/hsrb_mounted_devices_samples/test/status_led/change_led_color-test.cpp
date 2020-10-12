/// @brief C++ Change LED Color test
/// @brief Copyright (C) 2017 Toyota Motor Corporation
#include <vector>

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>

namespace {
const uint16_t kLEDInputDataSize = 256;
}  // unnamed namespace

namespace test_change_led_color {
class ChangeLEDColorTest : public ::testing::Test {
 public:
  ChangeLEDColorTest() {
    // Create Subscriber for status led topic
    ros::NodeHandle nh;
    led_sub = nh.subscribe(
        "/hsrb/command_status_led_rgb", 100,
        &ChangeLEDColorTest::LEDCallback,
        this);

    // Wait for connection
    while (led_sub.getNumPublishers() == 0) {
      ros::Duration(0.1).sleep();
    }
  }

  ~ChangeLEDColorTest() {
    led_sub.shutdown();
  }

  ros::Subscriber led_sub;

  // Hold subscribed time and rgb value of status led topic
  std::vector<ros::Time> time_vector;
  std::vector<std_msgs::ColorRGBA> led_value_vector;

 private:
  void LEDCallback(const std_msgs::ColorRGBAConstPtr& rgb) {
    time_vector.push_back(ros::Time::now());
    led_value_vector.push_back(*rgb);
  }
};

TEST_F(ChangeLEDColorTest, NormalSubscribeMsgTest) {
  // Wait until test node is finished
  while (led_sub.getNumPublishers() > 0) {
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }

  // R value check
  // r = x / (kLEDInputDataSize - 1)
  // {x | 0 <= x <= kLEDInputDataSize - 1}
  ASSERT_EQ(kLEDInputDataSize, led_value_vector.size());
  for (uint16_t x = 0; x < kLEDInputDataSize; x++) {
    ASSERT_FLOAT_EQ(x / static_cast<float>(kLEDInputDataSize - 1),
                    led_value_vector.at(x).r);
  }

  // Frequency check
  double duration = (time_vector.back() - time_vector.front()).toSec();
  double frequency = led_value_vector.size() / duration;
  ASSERT_NEAR(50, frequency, 1.0);
}
}  // namespace test_change_led_color


int main(int argc, char** argv) {
  ros::init(argc, argv, "test_change_led_color");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

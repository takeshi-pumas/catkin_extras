/// @brief C++ Suction controller test
/// @brief Copyright (C) 2017 Toyota Motor Corporation
#include "suction_controller_test_fixture.hpp"

namespace test_suction_controller {
TEST_F(SuctionControllerTest, SuctionFailedTest) {
  // Set parameter for setting stub's result
  ros::NodeHandle nh;
  nh.setParam("result_status", "ABORTED");

  // Wait for exit hsrb_suction_controller node
  ros::Duration(2.0).sleep();
  ros::spinOnce();

  ASSERT_EQ(SuctionControllerTest::goal_list.size(), 1);
  // goal_list_.at(0): suction_on request
  EXPECT_EQ(SuctionControllerTest::goal_list.at(0).timeout, ros::Duration(20.0));
  EXPECT_TRUE(SuctionControllerTest::goal_list.at(0).suction_on.data);

  ASSERT_EQ(SuctionControllerTest::log_message_list.size(), 2);
  EXPECT_EQ(SuctionControllerTest::log_message_list.at(0), "Suction will start");
  EXPECT_EQ(SuctionControllerTest::log_message_list.at(1), "Suction failed");
}
}  // namespace test_suction_controller

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_suction_controller");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

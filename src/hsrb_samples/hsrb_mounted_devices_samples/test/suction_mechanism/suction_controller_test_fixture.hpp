/// @brief C++ Suction controller's test fixture
/// @brief Copyright (C) 2017 Toyota Motor Corporation
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <rosgraph_msgs/Log.h>
#include <tmc_suction/SuctionControlGoal.h>


namespace test_suction_controller {
class SuctionControllerTest : public ::testing::Test {
 public:
  // Constructor
  SuctionControllerTest() : goal_list(),
                            log_message_list() {
    // Create goal subscriber to get suction server's input
    ros::NodeHandle nh;
    goal_sub_ = nh.subscribe("/goal_msg_for_test",
                             10,
                             &SuctionControllerTest::ServerGoalCallback,
                             this);

    // Create rosout subscriber to get node's output
    rosout_sub_ = nh.subscribe("/rosout",
                               10,
                               &SuctionControllerTest::RosoutCallback,
                               this);
    while (rosout_sub_.getNumPublishers() < 3) {
      ros::Duration(0.1).sleep();
      }
  }

  // Destructor
  ~SuctionControllerTest() {
    goal_sub_.shutdown();
    rosout_sub_.shutdown();
  }

  std::vector<tmc_suction::SuctionControlGoal> goal_list;
  std::vector<std::string> log_message_list;

 private:
  ros::Subscriber goal_sub_;
  ros::Subscriber rosout_sub_;

  void ServerGoalCallback(
      const tmc_suction::SuctionControlGoalConstPtr& action_goal) {
    goal_list.push_back(*action_goal);
  }

  void RosoutCallback(const rosgraph_msgs::LogConstPtr& log) {
    if (log->name == "/hsrb_suction_controller") {
      log_message_list.push_back(log->msg);
    }
  }
};
}  // namespace test_suction_controller

/// @brief Suction Controller Sample
/// @brief Copyright (C) 2017 Toyota Motor Corporation
#include <string>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_goal_state.h>
#include <ros/ros.h>
#include <tmc_suction/SuctionControlAction.h>
#include <tmc_suction/SuctionControlGoal.h>

namespace {
const double kConnectionTimeout = 10.0;

// Wait until pressure sensor is True
// If it is negative number, goal will be rejected
const ros::Duration kSuctionTimeout(20.0);
}  // unnamed namespace


int main(int argc, char** argv) {
  ros::init(argc, argv, "hsrb_suction_controller");

  // Create action client to control suction
  std::string suction_action = "/hsrb/suction_control";
  actionlib::SimpleActionClient<tmc_suction::SuctionControlAction>
      suction_control_client(suction_action, true);

  // Wait for connection
  if (!suction_control_client.waitForServer(ros::Duration(kConnectionTimeout))) {
    ROS_ERROR("%s does not exist", suction_action.c_str());
    return EXIT_FAILURE;
  }

  // Send a goal to start action
  ROS_INFO("Suction will start");
  tmc_suction::SuctionControlGoal suction_on_goal;
  suction_on_goal.timeout = kSuctionTimeout;
  suction_on_goal.suction_on.data = true;
  if (suction_control_client.sendGoalAndWait(suction_on_goal) ==
      actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Suction succeeded. Suction will stop");

    // Send a goal to stop suction
    tmc_suction::SuctionControlGoal suction_off_goal;
    suction_off_goal.suction_on.data = false;
    suction_control_client.sendGoalAndWait(suction_off_goal);
  } else {
    ROS_INFO("Suction failed");
  }

  return EXIT_SUCCESS;
}

/// @brief Speak Object Weight Sample
/// @brief Copyright (C) 2016 Toyota Motor Corporation
#include <cstdlib>
#include <math.h>
#include <string>

#include <boost/format.hpp>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/WrenchStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tmc_control_msgs/GripperApplyEffortAction.h>
#include <tmc_control_msgs/GripperApplyEffortGoal.h>
#include <tmc_manipulation_msgs/SafeJointChange.h>
#include <tmc_msgs/TalkRequestAction.h>
#include <tmc_msgs/TalkRequestGoal.h>
#include <tmc_msgs/Voice.h>

namespace {
const double kConnectionTimeout = 10.0;

// Create speak sentences
// Index 0: in Japanese, 1: in English
const char* kExplain1[2] = {"グリッパの間に重さをはかりたいものを持ってきてください",
                            "Please set the object between my gripper"};
const char* kExplain2[2] = {"グリッパを閉じます", "I close my hand now"};
const char* kAnswer[2] = {"これは%.1fグラムです", "This is %.1f gram"};

double ComputeDifference(const geometry_msgs::Vector3& pre_data,
                         const geometry_msgs::Vector3& post_data) {
  // Calculate square sum of difference
  double square_sums = pow(post_data.x - pre_data.x, 2) +
    pow(post_data.y - pre_data.y, 2) + pow(post_data.z - pre_data.z, 2);
  return sqrt(square_sums);
}
}  // unnamed namespace

namespace hsrb_mounted_devices_samples {
/// @class ForceSensorCapture
/// @brief Subscribe and hold force sensor data
class ForceSensorCapture {
 public:
  ForceSensorCapture() : force_data_() {
    // Subscribe force torque sensor data from HSRB
    ros::NodeHandle nh;
    std::string ft_sensor_topic = "/hsrb/wrist_wrench/raw";
    wrist_wrench_sub_ = nh.subscribe(ft_sensor_topic, 1,
                                     &ForceSensorCapture::FtSensorCb,
                                     this);

    // Wait for connection
    if (!ros::topic::waitForMessage<geometry_msgs::WrenchStamped>(
        ft_sensor_topic, ros::Duration(kConnectionTimeout))) {
      ROS_ERROR("timeout exceeded while waiting for message on topic %s",
                ft_sensor_topic.c_str());
      exit(EXIT_FAILURE);
    }
  }

  void GetCurrentForce(geometry_msgs::Vector3& force_data) const {
    // Spin FtSensorCb function once
    ros::spinOnce();
    force_data = force_data_;
  }

 private:
  ros::Subscriber wrist_wrench_sub_;
  geometry_msgs::Vector3 force_data_;

  void FtSensorCb(const geometry_msgs::WrenchStampedConstPtr& data) {
    force_data_ = data->wrench.force;
  }
};

/// @class JointController
/// @brief Control arm and gripper
class JointController {
 public:
  JointController()
      : gripper_control_client_("/hsrb/gripper_controller/grasp") {
    ros::NodeHandle nh;
    std::string joint_control_service = "/safe_pose_changer/change_joint";
    std::string grasp_action = "/hsrb/gripper_controller/grasp";
    joint_control_client_ =
        nh.serviceClient<tmc_manipulation_msgs::SafeJointChange>(
            joint_control_service);

    // Wait for connection
    if (!ros::service::waitForService(
        joint_control_service, ros::Duration(kConnectionTimeout))) {
      ROS_ERROR("timeout exceeded while waiting for service %s",
                joint_control_service.c_str());
      exit(EXIT_FAILURE);
    }

    if (!gripper_control_client_.waitForServer(
        ros::Duration(kConnectionTimeout))) {
      ROS_ERROR("%s does not exist", grasp_action.c_str());
      exit(EXIT_FAILURE);
    }
  }

  /// @brief Joint position control
  bool MoveToJointPositions(const sensor_msgs::JointState& goal_joint_states) {
    tmc_manipulation_msgs::SafeJointChange srv;
    srv.request.ref_joint_state = goal_joint_states;
    if (!joint_control_client_.call(srv)) {
      ROS_ERROR("failed to call joint control service");
      return false;
    }
    return srv.response.success;
  }

  /// @brief Gripper torque control
  bool Grasp(double effort) {
    tmc_control_msgs::GripperApplyEffortGoal goal;
    goal.effort = effort;

    // Send message to the action server
    if (gripper_control_client_.sendGoalAndWait(goal) ==
        actionlib::SimpleClientGoalState::SUCCEEDED) {
      return true;
    } else {
      return false;
    }
  }

 private:
  typedef actionlib::SimpleActionClient<
      tmc_control_msgs::GripperApplyEffortAction> GripperActionClient;

  ros::ServiceClient joint_control_client_;
  GripperActionClient gripper_control_client_;
};

/// @class Speaker
/// @brief Speak sentence in robot's language
class Speaker {
 public:
  Speaker() : talk_request_client_("/talk_request_action") {
    std::string talk_action = "/talk_request_action";

    // Wait for connection
    if (!talk_request_client_.waitForServer(
        ros::Duration(kConnectionTimeout))) {
      ROS_ERROR("%s does not exist", talk_action.c_str());
      exit(EXIT_FAILURE);
    }

    // Detect robot's language
    if (!strcmp(std::getenv("LANG"), "ja_JP.UTF-8")) {
      lang_ = tmc_msgs::Voice::kJapanese;
    } else {
      lang_ = tmc_msgs::Voice::kEnglish;
    }
  }

  int32_t GetLanguage() {
    return lang_;
  }

  bool SpeakSentence(const std::string& sentence) {
    tmc_msgs::TalkRequestGoal goal;
    goal.data.language = lang_;
    goal.data.sentence = sentence;

    if (talk_request_client_.sendGoalAndWait(goal) ==
        actionlib::SimpleClientGoalState::SUCCEEDED) {
      return true;
    } else {
      return false;
    }
  }

 private:
  typedef actionlib::SimpleActionClient<
      tmc_msgs::TalkRequestAction> TalkRequestActionClient;

  TalkRequestActionClient talk_request_client_;
  int32_t lang_;
};
}  // namespace hsrb_mounted_devices_samples

int main(int argc, char** argv) {
  ros::init(argc, argv, "hsrb_speak_object_weight");

  // Start force sensor capture
  hsrb_mounted_devices_samples::ForceSensorCapture force_sensor_capture;

  // Set initial pose
  hsrb_mounted_devices_samples::JointController joint_controller;

  sensor_msgs::JointState initial_position;
  initial_position.name.push_back("arm_lift_joint");
  initial_position.name.push_back("arm_flex_joint");
  initial_position.name.push_back("arm_roll_joint");
  initial_position.name.push_back("wrist_flex_joint");
  initial_position.name.push_back("wrist_roll_joint");
  initial_position.name.push_back("head_pan_joint");
  initial_position.name.push_back("head_tilt_joint");
  initial_position.name.push_back("hand_motor_joint");
  initial_position.position.push_back(0.0);
  initial_position.position.push_back(0.0);
  initial_position.position.push_back(0.0);
  initial_position.position.push_back(-1.57);
  initial_position.position.push_back(0.0);
  initial_position.position.push_back(0.0);
  initial_position.position.push_back(0.0);
  initial_position.position.push_back(1.2);
  joint_controller.MoveToJointPositions(initial_position);

  // Get initial data of force sensor
  geometry_msgs::Vector3 pre_force;
  force_sensor_capture.GetCurrentForce(pre_force);

  // Ask user to set object
  hsrb_mounted_devices_samples::Speaker speaker;
  speaker.SpeakSentence(kExplain1[speaker.GetLanguage()]);
  ros::Duration(2.0).sleep();

  // Inform user of next gripper action
  speaker.SpeakSentence(kExplain2[speaker.GetLanguage()]);
  ros::Duration(1.0).sleep();

  // Grasp the object
  joint_controller.Grasp(-0.1);

  // Wait until force sensor data become stable
  ros::Duration(1.0).sleep();
  geometry_msgs::Vector3 post_force;
  force_sensor_capture.GetCurrentForce(post_force);

  double force_difference = ComputeDifference(pre_force, post_force);

  // Convert newton to gram
  double weight = force_difference / 9.81 * 1000;

  // Speak object weight in first decimal place
  speaker.SpeakSentence((boost::format(kAnswer[speaker.GetLanguage()]) % weight).str());

  return 0;
}

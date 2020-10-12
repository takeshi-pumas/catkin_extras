/// @brief C++ Speak object weight test
/// @brief Copyright (C) 2016 Toyota Motor Corporation
#include <stdlib.h>
#include <string>
#include <vector>
#include <actionlib/server/simple_action_server.h>
#include <boost/thread.hpp>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/WrenchStamped.h>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <tmc_control_msgs/GripperApplyEffortAction.h>
#include <tmc_manipulation_msgs/SafeJointChange.h>
#include <tmc_msgs/TalkRequestAction.h>
#include <tmc_msgs/Voice.h>


namespace {
const double kUpdatedForceX = 5.0;
const double kUpdatedForceY = 0.1;
const double kUpdatedForceZ = 0.1;

const uint8_t kNumOfSentences = 3;
}  // unnamed namespace


namespace test_speak_object_weight {
class ROSInterfaceStub : public ::testing::Test {
 public:
  // Constructor
  ROSInterfaceStub()
      : force_(),
        update_force_(),
        lang_(0),
        sentences_(),
        node_exit_flag_(false),
        grasp_action_server_(nh_,
                             "/hsrb/gripper_controller/grasp",
                             boost::bind(&ROSInterfaceStub::GraspCallBack, this, _1),
                             false),
        talk_action_server_(nh_,
                            "/talk_request_action",
                            boost::bind(&ROSInterfaceStub::TalkRequestCallBack, this, _1),
                            false) {
    ft_sensor_pub =
        nh_.advertise<geometry_msgs::WrenchStamped>("/hsrb/wrist_wrench/raw", 1);

    joint_controller_srv_ =
        nh_.advertiseService("/safe_pose_changer/change_joint",
                             &ROSInterfaceStub::ChangeJointService,
                             this);
    grasp_action_server_.start();
    talk_action_server_.start();

    // Get language type
    if (!strcmp(std::getenv("LANG"), "ja_JP.UTF-8")) {
      lang_ = tmc_msgs::Voice::kJapanese;
    } else {
      lang_ = tmc_msgs::Voice::kEnglish;
    }
  }

  // Destructor
  ~ROSInterfaceStub() {}

  // Dummy publisher for ft sensor
  ros::Publisher ft_sensor_pub;

  void GetForceData(geometry_msgs::Vector3& dst_force) {
    dst_force = force_;
  }

  void SetUpdateForceData(const geometry_msgs::Vector3& update_force) {
    update_force_ = update_force;
  }

  int32_t GetLang() {
    return lang_;
  }

  void GetTalkSentences(std::vector<std::string>& dst_sentences) {
    dst_sentences = sentences_;
  }

  bool GetNodeExitFlag() {
    return node_exit_flag_;
  }

 private:
  ros::NodeHandle nh_;

  // Variable for publishing ft sensor data
  geometry_msgs::Vector3 force_;

  // Variable for publishing ft sensor updated data
  geometry_msgs::Vector3 update_force_;

  // Dummy Service Server for joint controller
  ros::ServiceServer joint_controller_srv_;

  // Dummy Action Server for grasp
  actionlib::SimpleActionServer<
    tmc_control_msgs::GripperApplyEffortAction> grasp_action_server_;

  // Dummy Action Server for talk request
  actionlib::SimpleActionServer<
    tmc_msgs::TalkRequestAction> talk_action_server_;

  // Speak language
  int32_t lang_;

  // Speak sentence
  std::vector<std::string> sentences_;

  // Node status flag
  // 0:running, 1:exit
  bool node_exit_flag_;

  void GraspCallBack(const tmc_control_msgs::GripperApplyEffortGoalConstPtr& goal) {
    // When robot grasp the object, ft sensor data is updated
    force_ = update_force_;

    grasp_action_server_.setSucceeded();
  }

  void TalkRequestCallBack(const tmc_msgs::TalkRequestGoalConstPtr& goal) {
    sentences_.push_back(goal->data.sentence);
    // Node will exit when all the requests are called
    if (sentences_.size() == kNumOfSentences) {
      node_exit_flag_ = true;
    }
    talk_action_server_.setSucceeded();
  }

  bool ChangeJointService(tmc_manipulation_msgs::SafeJointChange::Request& req,
                          tmc_manipulation_msgs::SafeJointChange::Response& res) {
    res.success = true;
    return true;
  }
};

TEST_F(ROSInterfaceStub, NormalTest) {
  // Set update force
  geometry_msgs::Vector3 update_force;
  update_force.x = kUpdatedForceX;
  update_force.y = kUpdatedForceY;
  update_force.z = kUpdatedForceZ;
  SetUpdateForceData(update_force);

  geometry_msgs::WrenchStamped wrench;
  ros::Rate rate(10);

  // Wait for testing node to end
  while (!GetNodeExitFlag()) {
    // Publish ft sensor data
    GetForceData(wrench.wrench.force);
    ft_sensor_pub.publish(wrench);
    ros::spinOnce();
    rate.sleep();
  }

  // Expect output is calculated by a below equation
  // sqrt((kUpdatedForceX-0.0)^2+(kUpdatedForceY-0.0)^2+(kUpdatedForceZ-0.0)^2))/9.81*1000
  const char* kAnswer[2] = {"これは509.9グラムです", "This is 509.9 gram"};

  // Check if the final output is correct
  std::vector<std::string> sentence;
  GetTalkSentences(sentence);
  EXPECT_EQ(std::string(kAnswer[GetLang()]), sentence.back());
}
}  // namespace test_speak_object_weight

int main(int argc, char** argv) {
  ros::init(argc, argv, "speak_object_weight_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

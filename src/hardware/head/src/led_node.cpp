#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tmc_msgs/SetColor.h>
#include "actionlib_msgs/GoalStatus.h"

const int loop_rate_hz = 5;

ros::ServiceClient client;

float mapToFloat(short int value) {
  return static_cast<float>(value) / 255.0;
}

void service_caller(float r, float g, float b) {
  // Create a request message for the service
  tmc_msgs::SetColor srv;
  srv.request.color.r = r;
  srv.request.color.g = g;
  srv.request.color.b = b;

  // Call the service
  if (client.isValid() && client.call(srv))
    ROS_INFO("Led color changed");
  else
    ROS_ERROR("Failed changing led color");
}

void nav_msg_Callback(const actionlib_msgs::GoalStatus::ConstPtr& msg) {
  float r, g, b;
  switch(msg -> status)
  {
    case actionlib_msgs::GoalStatus::ACTIVE:
      r = mapToFloat(255);
      g = r;
      b = 0.0;
      break;
    case actionlib_msgs::GoalStatus::SUCCEEDED:
      r = 0.0;
      g = mapToFloat(255);
      b = 0.0;
      break;
    case actionlib_msgs::GoalStatus::ABORTED:
      r = mapToFloat(139);
      g = 0.0;
      b = 0.0;
      break;
    case actionlib_msgs::GoalStatus::REJECTED:
      r = mapToFloat(85);
      g = r;
      b = r;
      break;
    default:
      ROS_WARN("Unknown goal status received");
      return;
  }
  service_caller(r, g, b);

}

void talk_now_Callback(const std_msgs::String::ConstPtr& msg) {
  std::string data = msg -> data;
  float r, g, b;
  if(data == "start"){
    r = mapToFloat(102);
    g = mapToFloat(204);
    b = mapToFloat(255);
  }
  else{
    r = 0.0;
    g = 0.0;
    b = 1.0;
  }
  service_caller(r, g, b);

}

int main(int argc, char** argv)
{
  // Initialize the ROS node
  ros::init(argc, argv, "service_client_node");
  ros::NodeHandle n("~");

  client = n.serviceClient<tmc_msgs::SetColor>("/hsrb/status_led_node/set_color");

  ros::Subscriber sub_nav_status  = n.subscribe("/navigation/status", 10, nav_msg_Callback);
  ros::Subscriber sub_talk_now    = n.subscribe("/talk_now", 10, talk_now_Callback);

  ros::Rate loop(loop_rate_hz);

  // while(ros::ok())
  // {
  //   ros::spinOnce();
  //   loop.sleep();
  // }
  ros::spin();
  

  return 0;
}

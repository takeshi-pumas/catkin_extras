#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <tmc_msgs/SetColor.h>
#include <iostream>
#include "actionlib_msgs/GoalStatus.h"

#define RATE 30

ros::ServiceClient client;

float mapToFloat(short int value)
{
  return static_cast<float>(value) / 255.0;
}

void nav_msg_Callback(const actionlib_msgs::GoalStatus::ConstPtr& msg)
{
  tmc_msgs::SetColor srv;
  float r;
  float g;
  float b;
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
  } 
  // Create a request message for the service
  srv.request.color.r = r;
  srv.request.color.g = g;
  srv.request.color.b = b;

  // Call the service
  if (client.call(srv))
  {
    // Handle the response
    // You can access the response using 'srv.response' and its fields

    // Example:
    std::cout << "Success" << std::endl;
    // ROS_INFO("Success!!");
  }
  else
  {
    // Handle the case when the service call failed
    std::cout << "Failed" << std::endl;
  }
}

int main(int argc, char** argv)
{
  // Initialize the ROS node
  ros::init(argc, argv, "service_client_node");
  ros::NodeHandle n("~");

  client = n.serviceClient<tmc_msgs::SetColor>("/hsrb/status_led_node/set_color");

  ros::Subscriber sub = n.subscribe("/navigation/status", 10, nav_msg_Callback);

  ros::Rate loop(RATE);
  while(ros::ok())
  {
    ros::spinOnce();
    loop.sleep();
  }
  

  return 0;
}

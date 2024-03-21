#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <cstdlib>

void micControlCallback(const std_msgs::Bool::ConstPtr& msg) {
    if (msg->data) {
        system("pactl set-source-mute @DEFAULT_SOURCE@ 0");
    } else {
        system("pactl set-source-mute @DEFAULT_SOURCE@ 1");
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "mic_controller");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/talk_now", 1, micControlCallback);

    //Muted by default
    system("pactl set-source-mute @DEFAULT_SOURCE@ 1");

    ros::spin();

    return 0;
}

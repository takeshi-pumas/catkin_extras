#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>
#include <vector>
#include <cmath>

ros::NodeHandle* nh;
ros::Publisher pub_marker_array;
ros::Publisher pub_active_waypoint;
ros::Subscriber sub_legs_pose;
ros::Subscriber sub_next_waypoint;

tf::TransformListener* tf_listener;
ros::Timer timer;
std::vector<std::pair<float, float>> leg_positions;
std::pair<float, float> last_leg_position;
bool new_legs_pose = false;
bool enable = false;
unsigned int active_waypoint_index = 0;


visualization_msgs::Marker create_waypoint(float posx, float posy, int id) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "legs";
    marker.id = id;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = posx;
    marker.pose.position.y = posy;
    marker.pose.position.z = 0.3;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.lifetime = ros::Duration();
    return marker;
}

void publish_active_waypoint() {
    if (!leg_positions.empty() && active_waypoint_index < leg_positions.size()) {
        geometry_msgs::PointStamped active_waypoint;
        active_waypoint.header.frame_id = "map";
        active_waypoint.header.stamp = ros::Time::now();
        active_waypoint.point.x = leg_positions[active_waypoint_index].first;
        active_waypoint.point.y = leg_positions[active_waypoint_index].second;
        active_waypoint.point.z = 0.1;
        pub_active_waypoint.publish(active_waypoint);
    }
}

void legs_pose_callback(const geometry_msgs::PointStamped::ConstPtr& msg) {
    geometry_msgs::PointStamped point_in_map;
    try {
        tf_listener->transformPoint("map", *msg, point_in_map);
    } catch (tf::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        return;
    }
    
    float human_x = point_in_map.point.x;
    float human_y = point_in_map.point.y;
    last_leg_position = std::make_pair(human_x, human_y);
    new_legs_pose = true;
}

void next_waypoint_callback(const std_msgs::Empty::ConstPtr&) {
    if (!leg_positions.empty()) {
        if (active_waypoint_index < leg_positions.size() - 1) {
            active_waypoint_index++;
        }
        publish_active_waypoint();
    }
}

void timer_callback(const ros::TimerEvent&) {
    if (new_legs_pose) {
        new_legs_pose = false;

        if (!leg_positions.empty()) {
            float distance_between_waypoints = std::sqrt((last_leg_position.first - leg_positions.back().first) * 
                                                         (last_leg_position.first - leg_positions.back().first) + 
                                                         (last_leg_position.second - leg_positions.back().second) * 
                                                         (last_leg_position.second - leg_positions.back().second));

            if (distance_between_waypoints > 0.25) {
                ROS_INFO("New waypoint added");
                leg_positions.push_back(last_leg_position);
            }
        } else {
            leg_positions.push_back(last_leg_position);
        }

        visualization_msgs::MarkerArray waypoints;
        int num_waypoints = leg_positions.size();
        waypoints.markers.resize(num_waypoints);
        for (size_t i = 0; i < num_waypoints; ++i) {
            waypoints.markers[i] = create_waypoint(leg_positions[i].first, leg_positions[i].second, i);
        }
        pub_marker_array.publish(waypoints);
    }
}

void callback_enable(const std_msgs::Bool::ConstPtr& msg) {
    if(msg->data)
    {
        std::cout << "LegFinder.->Enable recevied" << std::endl;
        sub_legs_pose = nh->subscribe("/hri/leg_finder/leg_pose", 1, legs_pose_callback);
        sub_next_waypoint = nh->subscribe("/hri/waypoints/next_waypoint", 1, next_waypoint_callback);
        timer = nh->createTimer(ros::Duration(2.0), timer_callback);  
    }
    else
    {
        sub_legs_pose.shutdown();
        sub_next_waypoint.shutdown();
        timer.stop();
    }
    enable = msg->data;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "waypoints_node");
    nh = new ros::NodeHandle();

    tf_listener = new tf::TransformListener();

    
    ros::Subscriber sub_enable = nh->subscribe("/hri/human_following/start_follow", 1, callback_enable);
    pub_marker_array = nh->advertise<visualization_msgs::MarkerArray>("/hri/human_follower/waypoints", 1);
    pub_active_waypoint = nh->advertise<geometry_msgs::PointStamped>("/hri/human_follower/active_waypoint", 1);

    ros::spin();

    delete tf_listener;
    return 0;
}

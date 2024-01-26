#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>

class GazeController {
public:
    GazeController() : nh_("~"), tf_listener_(tf_buffer_) {
        // Suscribirse al tópico "/gaze/absolute"
        sub_gaze_absolute_ = nh_.subscribe("/gaze/absolute", 1, &GazeController::absoluteGazeCallback, this);

        // Suscribirse al tópico "/gaze/relative"
        sub_gaze_relative_ = nh_.subscribe("/gaze/relative", 1, &GazeController::relativeGazeCallback, this);

        // Suscribirse al tópico "/gaze/relative"
        sub_gaze_tf_ = nh_.subscribe("/gaze/tf", 1, &GazeController::TFGazeCallback, this);

        // Publicar los ángulos de PAN y TILT en el tópico "/hardware/head/goal_pose"
        pub_head_control_ = nh_.advertise<std_msgs::Float32MultiArray>("/hardware/head/goal_pose", 10);
    }

    void absoluteGazeCallback(const std_msgs::Float32MultiArray::ConstPtr& gaze_msg) {
        // Calcular ángulos PAN y TILT con respecto a MAP
        ROS_INFO("Map absolute gaze");
        gazePoint(gaze_msg->data, "map");
    }

    void relativeGazeCallback(const std_msgs::Float32MultiArray::ConstPtr& gaze_msg) {
        // Calcular ángulos PAN y TILT con respecto a base_link
        ROS_INFO("Base link relative gaze");

        geometry_msgs::PointStamped point;

        point.header.frame_id = "base_link";
        point.point.x = gaze_msg->data[0]; 
        point.point.y = gaze_msg->data[1];
        point.point.z = gaze_msg->data[2];

        tf_buffer_.transform(point, point, "map", ros::Duration(0.3));
        std::vector<float> target;
        target.push_back(point.point.x);
        target.push_back(point.point.y);
        target.push_back(point.point.z);
        gazePoint(target, "map");
    }

    void TFGazeCallback(const std_msgs::String::ConstPtr& tf_name_msg){
        ROS_INFO("Gaze to a published tf");
        geometry_msgs::TransformStamped tf_position;
        tf_position = tf_buffer_.lookupTransform("map", tf_name_msg->data, ros::Time(0), ros::Duration(0.3));
        std::vector<float> target;
        target.push_back(tf_position.transform.translation.x);
        target.push_back(tf_position.transform.translation.y);
        target.push_back(tf_position.transform.translation.z);
        gazePoint(target, "map");
    }

    void gazePoint(const std::vector<float>& target_position, const std::string& ref_frame){
        try{
            geometry_msgs::TransformStamped trans;
            geometry_msgs::TransformStamped rot;
            double yaw = 0.0;

            trans = tf_buffer_.lookupTransform(ref_frame, "head_rgbd_sensor_link", ros::Time(0), ros::Duration(0.3));

            if (ref_frame == "map"){
                rot = tf_buffer_.lookupTransform(ref_frame, "base_link", ros::Time(0), ros::Duration(0.3));
                tf2::Quaternion quat(rot.transform.rotation.x, rot.transform.rotation.y, rot.transform.rotation.z, rot.transform.rotation.w);
                double roll, pitch;
                tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
            }

            double x_rob = trans.transform.translation.x;
            double y_rob = trans.transform.translation.y;
            double z_rob = trans.transform.translation.z;

            double D_x = x_rob - target_position[0];
            double D_y = y_rob - target_position[1];
            double D_z = z_rob - target_position[2];
            double D_th = atan2(D_y, D_x);

            double pan = fmod((-yaw + D_th + M_PI), (2*M_PI));
            double tilt = -atan2(D_z, sqrt(D_x * D_x + D_y * D_y));

            std_msgs::Float32MultiArray head_control_msg;
            head_control_msg.data.push_back(pan);
            head_control_msg.data.push_back(tilt);

            ROS_INFO("Publishing head angles: %f, %f", pan, tilt);
            pub_head_control_.publish(head_control_msg);
        } catch (tf2::TransformException& ex) {
            ROS_ERROR("Error al transformar la posicion objetivo: %s", ex.what());
        }
    }

private:
    ros::NodeHandle nh_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    ros::Subscriber sub_gaze_absolute_;
    ros::Subscriber sub_gaze_relative_;
    ros::Subscriber sub_gaze_tf_;
    ros::Publisher pub_head_control_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "gaze_controller_node");
    ROS_INFO("Gaze controller node initialized");

    GazeController gaze_controller;

    ros::spin();

    return 0;
}

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class GazeController {
public:
    GazeController() : nh_("~"), tf_listener_(tf_buffer_) {
        // Suscribirse al tópico "/gaze/absolute"
        sub_gaze_absolute_ = nh_.subscribe("/gaze/absolute", 1, &GazeController::absoluteGazeCallback, this);

        // Suscribirse al tópico "/gaze/relative"
        sub_gaze_relative_ = nh_.subscribe("/gaze/relative", 1, &GazeController::relativeGazeCallback, this);

        // Publicar los ángulos de PAN y TILT en el tópico "/hardware/head/goal_pose"
        pub_head_control_ = nh_.advertise<std_msgs::Float32MultiArray>("/hardware/head/goal_pose", 10);
    }

    void absoluteGazeCallback(const std_msgs::Float32MultiArray::ConstPtr& gaze_msg) {
        // Calcular ángulos PAN y TILT con respecto a MAP
        calculatePanTilt(gaze_msg->data, "map");
    }

    void relativeGazeCallback(const std_msgs::Float32MultiArray::ConstPtr& gaze_msg) {
        // Calcular ángulos PAN y TILT con respecto a base_link
        calculatePanTilt(gaze_msg->data, "base_link");
    }

    void calculatePanTilt(const std::vector<float>& target_position, const std::string& target_frame) {
        geometry_msgs::PointStamped target_point;
        target_point.header.frame_id = target_frame;
        target_point.point.x = target_position[0];
        target_point.point.y = target_position[1];
        target_point.point.z = target_position[2];

        try {
            // Transformar la posición al marco 'head_rgbd_sensor_link'
            geometry_msgs::PointStamped target_point_head_link;
            tf_buffer_.transform(target_point, target_point_head_link, "head_rgbd_sensor_link");

            // Calcular los ángulos PAN y TILT
            double pan = atan2(target_point_head_link.point.y, target_point_head_link.point.x);
            double tilt = atan2(-target_point_head_link.point.z,
                                sqrt(target_point_head_link.point.x * target_point_head_link.point.x +
                                     target_point_head_link.point.y * target_point_head_link.point.y));

            // Publicar los ángulos en el tópico "/hardware/head/goal_pose"
            std_msgs::Float32MultiArray head_control_msg;
            head_control_msg.data.push_back(pan);
            head_control_msg.data.push_back(tilt);

            std::cout << "Head angles are: " << pan << "," << tilt << std::endl;
            pub_head_control_.publish(head_control_msg);
        } catch (tf2::TransformException& ex) {
            ROS_ERROR("Error al transformar la posición objetivo: %s", ex.what());
        }
    }

private:
    ros::NodeHandle nh_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    ros::Subscriber sub_gaze_absolute_;
    ros::Subscriber sub_gaze_relative_;
    ros::Publisher pub_head_control_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "gaze_controller_node");

    GazeController gaze_controller;

    ros::spin();

    return 0;
}

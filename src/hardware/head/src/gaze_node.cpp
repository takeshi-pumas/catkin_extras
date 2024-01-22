#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/tf.h>
#include <cmath>

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
        //calculatePanTilt(gaze_msg->data, "map");
        gazePoint(gaze_msg->data, "map");
    }

    void relativeGazeCallback(const std_msgs::Float32MultiArray::ConstPtr& gaze_msg) {
        // Calcular ángulos PAN y TILT con respecto a base_link
        //calculatePanTilt(gaze_msg->data, "base_link");
        gazePoint(gaze_msg->data, "base_link");
    }

    void calculatePanTilt(const std::vector<float>& target_position, const std::string& target_frame) {
        geometry_msgs::PointStamped target_point;
        target_point.header.frame_id = target_frame;
        target_point.point.x = target_position[0];
        target_point.point.y = target_position[1];
        target_point.point.z = target_position[2];

        try {
            // Transformar la posición al marco 'head_rgbd_sensor_link'
            geometry_msgs::PointStamped target_tilt;
            geometry_msgs::PointStamped target_pan;
            tf_buffer_.transform(target_point, target_pan, "base_link");
            tf_buffer_.transform(target_point, target_tilt, "head_pan_link");

            // Calcular los ángulos PAN y TILT
            double pan = atan2(target_pan.point.y, target_pan.point.x);
            
            double tilt = atan2(-target_tilt.point.z,
                                sqrt(target_tilt.point.x * target_tilt.point.x +
                                     target_tilt.point.y * target_tilt.point.y));

            // Publicar los ángulos en el tópico "/hardware/head/goal_pose"
            std_msgs::Float32MultiArray head_control_msg;
            head_control_msg.data.push_back(tilt);
            head_control_msg.data.push_back(pan);

            ROS_INFO("Head angles are: %f, %f", pan, tilt);
            pub_head_control_.publish(head_control_msg);
        } catch (tf2::TransformException& ex) {
            ROS_ERROR("Error al transformar la posicion objetivo: %s", ex.what());
        }
    }

    void gazePoint(const std::vector<float>& target_position, const std::string& ref_frame){
        try{
            geometry_msgs::TransformStamped trans;
            geometry_msgs::TransformStamped rot;

            trans = tf_buffer_.lookupTransform("head_rgbd_sensor_link", ref_frame,  ros::Time(0), ros::Duration(0.3));
            rot = tf_buffer_.lookupTransform("base_link", ref_frame, ros::Time(0), ros::Duration(0.3));
            tf::Quaternion quat(rot.transform.rotation.x, rot.transform.rotation.y, rot.transform.rotation.z, rot.transform.rotation.w);
            double roll, pitch, yaw;
            tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
            
            double x_rob = trans.transform.translation.x;
            double y_rob = trans.transform.translation.y;
            double z_rob = trans.transform.translation.z;

            double D_x = x_rob - target_position[0];
            double D_y = y_rob - target_position[1];
            double D_z = z_rob - target_position[2];
            double D_th = atan2(D_y, D_x);

            double pan = fmod((-yaw + D_th + M_PI), (M_PI));
            double tilt = -atan2(D_z, sqrt(D_x * D_x + D_y * D_y));

            std_msgs::Float32MultiArray head_control_msg;
            head_control_msg.data.push_back(tilt);
            head_control_msg.data.push_back(pan);

            ROS_INFO("Head angles are: %f, %f", pan, tilt);
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
    ros::Publisher pub_head_control_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "gaze_controller_node");

    GazeController gaze_controller;

    ros::spin();

    return 0;
}

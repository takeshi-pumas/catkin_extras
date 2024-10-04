#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float32MultiArray.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class GazeController {
public:
    GazeController() : tfListener(tfBuffer) {
        nh.param<std::string>("head_frame", head_frame, "head_rgbd_sensor_link");
        nh.param<std::string>("reference_frame", reference_frame, "base_link"); // Frame de referencia

        // Posicion de la cabeza con respecto a base_link
        head_offset_z = getHeadOffsetZ();

        pub_head_controller = nh.advertise<std_msgs::Float32MultiArray>("/hardware/head/goal_pose", 10);
        sub_gaze_position = nh.subscribe(sub_gaze_position_topic, 1, &GazeController::gazeCallback, this);
    }

    void gazeCallback(const geometry_msgs::PointStamped::ConstPtr& gaze_msg) {
        lookAt(*gaze_msg);
    }

    void lookAt(const geometry_msgs::PointStamped& target_point) {
        try {
            // Transformar el punto objetivo al frame de referencia (base_link o map)
            geometry_msgs::PointStamped point_in_reference_frame;
            tfBuffer.transform(target_point, point_in_reference_frame, reference_frame, ros::Duration(1.0));

            // Ajustar la posición del punto considerando el offset de la cabeza en Z
            point_in_reference_frame.point.z -= head_offset_z;

            // Calcular los ángulos de pan y tilt en función de la referencia (no de la cabeza actual)
            double pan_angle = atan2(point_in_reference_frame.point.y, point_in_reference_frame.point.x);
            double distance = sqrt(pow(point_in_reference_frame.point.x, 2) + pow(point_in_reference_frame.point.y, 2));
            double tilt_angle = atan2(point_in_reference_frame.point.z, distance);

            // Publicar los comandos de pan y tilt
            std_msgs::Float32MultiArray head_msg;
            head_msg.data.resize(2);
            head_msg.data[0] = pan_angle;  // Pan
            head_msg.data[1] = tilt_angle; // Tilt
            pub_head_controller.publish(head_msg);

            ROS_INFO("Head moving to Pan: %.2f, Tilt: %.2f", pan_angle, tilt_angle);

        } catch (tf2::TransformException &ex) {
            ROS_WARN("Transform error: %s", ex.what());
        }
    }
    
    double getHeadOffsetZ() {

        for(int attempts = 0; attempts <= 5; ++attempts ){
            try {
                // Obtener la transformación de base a cabeza
                geometry_msgs::TransformStamped transform = tfBuffer.lookupTransform(reference_frame, head_frame, ros::Time(0), ros::Duration(5.0));
                ROS_INFO("Initialization completed, ready to use. \n Topic: %s", sub_gaze_position_topic.c_str());
                return transform.transform.translation.z;
            } catch (tf2::TransformException &ex) {
                ROS_WARN("Transform error during initialization: %s \n Retriyng (%d / 5)", ex.what(), attempts);
            }
        }
        // return 0.0; // Valor por defecto en caso de error
        ROS_ERROR("Failed to get initial transform, please restart node.");
        ros::shutdown();
    }

private:
    ros::NodeHandle nh;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;
    ros::Publisher pub_head_controller;
    ros::Subscriber sub_gaze_position;
    std::string head_frame;
    std::string reference_frame;
    double head_offset_z;
    std::string sub_gaze_position_topic = "/hardware/head/gaze";
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "gaze_controller");

    GazeController gazeController;

    ros::spin();

    return 0;
}

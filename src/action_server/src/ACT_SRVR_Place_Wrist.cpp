#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Float32MultiArray.h>

class ArmController {
private:
    ros::NodeHandle nh_;
    ros::Subscriber wrench_sub_;
    ros::Subscriber current_pose_sub_;
    ros::Publisher arm_pub_;
    float threshold_;
    std::vector<float> current_pose_;

public:
    ArmController() : nh_("~") {
        nh_.param<float>("collision_threshold", threshold_, 10.0); // Umbral de colisión (puedes ajustarlo)
        wrench_sub_ = nh_.subscribe("/hsrb/wrist_wrench/compensated", 1, &ArmController::wrenchCallback, this);
        current_pose_sub_ = nh_.subscribe("/hardware/arm/current_pose", 1, &ArmController::poseCallback, this);
        arm_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/hardware/arm/goal_pose", 1);
    }

    void poseCallback(const std_msgs::Float32MultiArray::ConstPtr& pose_msg) {
        current_pose_ = pose_msg->data;
    }

    void wrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr& wrench_msg) {
        // Verificar si el valor del sensor de fuerza supera el umbral
        if (wrench_msg->wrench.force.z > threshold_) {
            // Aquí puedes agregar la lógica para bajar el brazo del robot
            // Por ejemplo, publicar un mensaje en el topic "/hardware/arm/goal_pose"
            std_msgs::Float32MultiArray arm_msg;
            // Supongamos que el movimiento vertical del brazo es -0.5 (ajusta según tu robot)
            arm_msg.data = {float(current_pose_[0] - 0.1), current_pose_[1], current_pose_[2], current_pose_[3]};
            arm_pub_.publish(arm_msg);
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "arm_controller_node");
    ArmController controller;
    ros::spin();
    return 0;
}


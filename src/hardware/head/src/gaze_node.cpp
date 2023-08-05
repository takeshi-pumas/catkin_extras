#include <ros/ros.h>

// c++ libraries
#include <cmath>
#include <vector>
#include <math.h>
#include <iostream>

// TF2 functionalities
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

// ROS messages
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
// #include <trajectory_msgs/JointTrajectory.h>
// #include <trajectory_msgs/JointTrajectoryPoint.h>
// #include <sensor_msgs/JointState.h>




// TF2_ros objects
tf2_ros::Buffer tfbuffer;
//tf2_ros::TransformListener; // listener(tfbuffer);
// ---- GAZE class ----

class Gaze {
private:  
    double x_target = 0;
    double y_target = 0;
    double z_target = 0;
    std::string map = "map";
    std::string cam = "head_rgbd_sensor_link";
    std::string base = "base_link";
    std::string source;

        
    // Get transform from tf2_ros library function
    std::vector<double> getTF(std::string target_frame, std::string source_frame){

        try {
            geometry_msgs::TransformStamped trans;
            trans = tfbuffer.lookupTransform(target_frame, source_frame, ros::Time::now());

            // Use the transform information
            double x = trans.transform.translation.x;
            double y = trans.transform.translation.y;
            double z = trans.transform.translation.z;
            double roll, pitch, yaw;
            tf2::Quaternion q(trans.transform.rotation.x,
                              trans.transform.rotation.y,
                              trans.transform.rotation.z,
                              trans.transform.rotation.w);
            tf2::Matrix3x3 m(q);
            m.getRPY(roll, pitch, yaw);

            std::vector<double> v = {x,y,z,roll,pitch,yaw};
            return v;

        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            // Handle the exception (e.g., transform not available at the requested time)
             std::vector<double> v = {0};
            return v;
        }
    }

    // Stablish target point to calculate pan and tilt angles
    std::vector<double> gaze_abs_rel(double x, double y, double z) {
        this -> x_target = x;
        this -> y_target = y;
        this -> z_target = z;
        std::vector<double> head_pose = this->gaze_point();
        return head_pose;
    }
    
    // Calculates pan and tilt angles
    std::vector<double> gaze_point() {
        // Moves head to make center point of rgbd image to coordinates w.r.t.map
        double x_rob, y_rob, z_rob, th_rob;
        std::vector<double> cam_pos = this->getTF(this->cam, this->map);
        std::vector<double> rob_rot = this->getTF(this->base, this->map);

        // maybe this is not going to be used
        // ros::Duration(0.3).sleep();

        x_rob = cam_pos[0];
        y_rob = cam_pos[1];
        z_rob = cam_pos[2];
        th_rob = rob_rot[5];
        
        double D_x = x_rob - this->x_target;
        double D_y = y_rob - this->y_target;
        double D_z = z_rob - this->z_target;
        double D_th = std::atan2(D_y, D_x);
        // double pan_correct = (-th_rob + D_th + M_PI) % (2 * M_PI);

        double pan_correct = std::fmod((-th_rob + D_th + M_PI), (2 * M_PI));
        
        if (pan_correct > M_PI)
            pan_correct -= 2 * M_PI;
        else if (pan_correct < -M_PI)
            pan_correct += 2 * M_PI;

        double tilt_correct = -std::atan2(D_z, std::sqrt(D_x * D_x + D_y * D_y));

        if (std::abs(pan_correct) > 0.5 * M_PI) {
            std::cout << "Exorcist alert" << std::endl;
            std::vector<double> head_pose = {0.0, tilt_correct};
            return head_pose;
            // turn_base_gaze(); <---- It's needed to implement this function
        } else {
            std::vector<double> head_pose = {pan_correct, tilt_correct};
            return head_pose;
        }
    }



public:
    //GAZE() {
        // The class is not going to publish anything. 
        // It's just returning the tilt and pan angles to publish them to "head" ROS NODE

    //}

    // Stablishes "map" as source frame
    std::vector<double> absolute(double x, double y, double z) {
        // Head gaze to a x, y, z point relative to map
        this -> source = "map";
        return this->gaze_abs_rel(x, y, z);
    }

    // Stablishes "base_link" as source frame 
    std::vector<double> relative(double x, double y, double z) {
        // Head gaze to a x, y, z point relative to base_link
        this -> source = "base_link";
        return this->gaze_abs_rel(x, y, z);
    }

    // Pre determined head poses (needs callback implementation)
    std::vector<double> set_named_target(std::string pose = "neutral") {
        std::vector<double> head_pose;
        if (pose == "down") {
            head_pose = {0.0, -1.0};
        } else if (pose == "up") {
            head_pose = {0.0, 1.0};
        } else if (pose == "right") {
            head_pose = {0.7, 0.0};
        } else if (pose == "left") {
            head_pose = {-0.7, 0.0};
        } else {
            head_pose = {0.0, 0.0};
        }
        // set_joint_values(head_pose);
        return head_pose;
    }

    //  ---------------- Needs implementation 
    // std::vector<double> get_joint_values() {
    //     ros::Subscriber sub = nh.subscribe("/hsrb/joint_states", 1, &GAZE::jointStatesCallback, this);
    //     ros::Rate rate(10); // 10 Hz
    //     while (joint_states.position.size() < 11 && ros::ok()) {
    //         ros::spinOnce();
    //         rate.sleep();
    //     }
    //     return {joint_states.position[9], joint_states.position[10]};
    // }

    // void set_joint_values(std::vector<double> head_pose = {0.0, 0.0}) {
    //     // Fill ROS message
    //     trajectory_msgs::JointTrajectory traj;
    //     traj.joint_names = {"head_pan_joint", "head_tilt_joint"};
    //     trajectory_msgs::JointTrajectoryPoint p;
    //     p.positions = head_pose;
    //     p.velocities = {0.1, 0.1};
    //     p.time_from_start = ros::Duration(0.07);
    //     traj.points = {p};

    //     // Publish ROS message
    //     _pub.publish(traj);
    // }

    // Gazes to a published transform
    std::vector<double> to_tf(std::string target_frame) {
        geometry_msgs::TransformStamped xyz;
        std::vector<double> tf_target = this->getTF(target_frame, this->map);
        std::vector<double> head_pose = this->absolute(tf_target[0], tf_target[1], tf_target[2]);
        return head_pose;
    }

    // void turn_base_gaze(std::string tf = "None", std::string to_gaze = "base_link") {
    //     OMNIBASE base;
    //     bool succ = false;
    //     const double THRESHOLD = 0.05;
    //     int tries = 0;
    //     std::string target_frame;
    //     if (tf != "None") {
    //         target_frame = tf;
    //     } else {
    //         target_frame = "gaze";
    //         tf::StampedTransform gaze_transform;
    //         gaze_transform.getOrigin().setX(_x);
    //         gaze_transform.getOrigin().setY(_y);
    //         gaze_transform.getOrigin().setZ(_z);
    //         _tf_man.pub_static_tf(gaze_transform, "gaze");
    //         ros::Duration(0.1).sleep();
    //     }

    //     while (!succ && tries <= 10) {
    //         tries++;
    //         ros::Duration(0.2).sleep();
    //         tf::StampedTransform xyz;
    //         _tf_man.getTF(to_gaze, target_frame, xyz);
    //         double eT = 0;

    //         if (xyz.getOrigin().x() != 0 || xyz.getOrigin().y() != 0) {
    //             eT = std::atan2(xyz.getOrigin().y(), xyz.getOrigin().x());
    //             succ = std::abs(eT) < THRESHOLD;
    //         }

    //         if (succ) {
    //             eT = 0;
    //         }

    //         base.tiny_move(eT, 0.9);
    //     }
    // }
};

// ---------------



// head pose published (to "head" ROS NODE)
ros::Publisher pub_pumas_head_gp;

void pub_head_pose(std::vector<double> head_pose);
void absCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
void relCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
void tfCallback(const std_msgs::String::ConstPtr& msg);



int main(int argc, char **argv)
{
    std::cout << std::endl << "--------------------->" << std::endl;
    std::cout << "INITIALIZING HEAD_GAZE_NODE BY RUSANROD" << std::endl;
    ros::init(argc, argv, "gaze_node");

    // Instance of gaze class
    
    tf2_ros::TransformListener listener(tfbuffer);
    ros::NodeHandle n;
    ros::Subscriber sub_gaze_abs    = n.subscribe("/hardware/head/gaze/absolute", 10, absCallback);
    ros::Subscriber sub_gaze_rel    = n.subscribe("/hardware/head/gaze/relative", 10, relCallback);
    ros::Subscriber sub_gaze_tf     = n.subscribe("/hardware/head/gaze/toTF", 10, tfCallback);

    pub_pumas_head_gp = n.advertise<std_msgs::Float32MultiArray>("/hardware/head/goal_pose", 10);

    ros::Rate rate(10.0);
    while(ros::ok())
    {

        rate.sleep();
        ros::spinOnce();
    }
    return 0;

}

void pub_head_pose(std::vector<double> head_pose){
    std_msgs::Float32MultiArray msg;
    msg.data.resize(2);
    msg.data[0] = head_pose[0];
    msg.data[1] = head_pose[1];
    pub_pumas_head_gp.publish(msg);
}

void absCallback(const std_msgs::Float32MultiArray::ConstPtr& msg){
    Gaze head;
    std::size_t size = msg->data.size();
    if(size == 3){
        std::vector<double> head_pose = head.absolute(msg->data[0], msg->data[1], msg->data[2]);
        pub_head_pose(head_pose);
    }
}

void relCallback(const std_msgs::Float32MultiArray::ConstPtr& msg){
    Gaze head;
    std::size_t size = msg->data.size();
    if(size == 3){
        std::vector<double> head_pose = head.relative(msg->data[0], msg->data[1], msg->data[2]);
        pub_head_pose(head_pose);
    }
}

void tfCallback(const std_msgs::String::ConstPtr& msg){
    Gaze head;
    // std::size_t size = msg->data.size();
    std::vector<double> head_pose = head.to_tf(msg->data);
    pub_head_pose(head_pose);
}

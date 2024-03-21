#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <geometry_msgs/TransformStamped.h>



class PointCloudTransformer {
public:
    PointCloudTransformer() : nh_("~") {
        input_topic_ = "/hsrb/head_rgbd_sensor/depth_registered/rectified_points";
        output_topic_ = "/point_cloud_transformed";

        //sub_ = nh_.subscribe(input_topic_, 1, &PointCloudTransformer::pointCloudCallback, this);
        //pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_, 1);

        tf_listener_ = new tf2_ros::TransformListener(tf_buffer_);
    }

/*void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
        sensor_msgs::PointCloud2 transformed_cloud;
        geometry_msgs::TransformStamped transform;

        // Intenta obtener la transformación necesaria
        try {
            transform = tf_buffer_.lookupTransform("map", msg->header.frame_id, ros::Time(0), ros::Duration(1));
        } catch (tf2::TransformException& ex) {
            ROS_WARN("Transform exception: %s", ex.what());
            return;
        }

        // Aplica la transformación a la nube de puntos
        try {
            tf2::doTransform(*msg, transformed_cloud, transform);
            
                        // Filtra las lecturas que estén debajo del umbral
            sensor_msgs::PointCloud2 filtered_cloud;
            pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
            pcl::fromROSMsg(transformed_cloud, pcl_cloud);
            pcl::PointCloud<pcl::PointXYZ> filtered_pcl_cloud;

            for (const auto& point : pcl_cloud.points) {
                if (point.z >= filter_threshold_) {
                    filtered_pcl_cloud.points.push_back(point);
                }
            }

            pcl::toROSMsg(filtered_pcl_cloud, filtered_cloud);

            // Publica la nube de puntos filtrada
            pub_.publish(filtered_cloud);
        } catch (tf2::TransformException& ex) {
            ROS_WARN("Transform exception: %s", ex.what());
        }
    }*/

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener* tf_listener_;
    std::string input_topic_;
    std::string output_topic_;
    double filter_threshold_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "point_cloud_transformer_node");
    PointCloudTransformer pc_transformer;
    ros::spin();
    return 0;
}

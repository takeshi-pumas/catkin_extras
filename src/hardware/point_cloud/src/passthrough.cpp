#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>

ros::Publisher pub;
tf2_ros::Buffer tf_buffer;



bool transformPointCloud(const std::string& target_frame, const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud_in,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out) {
    static tf2_ros::TransformListener tf_listener(tf_buffer);
    geometry_msgs::TransformStamped transformStamped;
    try {
        transformStamped = tf_buffer.lookupTransform(target_frame, cloud_in->header.frame_id, ros::Time(0));
    } catch (tf2::TransformException& ex) {
        ROS_WARN("%s", ex.what());
        return false;
    }

    //tf2::doTransform(*cloud_in, *cloud_out, transformStamped);
    pcl_ros::transformPointCloud(target_frame, *cloud_in, *cloud_out, tf_buffer);
    return true;
}

void filterPointCloudZ(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud_in,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out) {
    double z_min = 0.03; // Valor mínimo en coordenada Z (base_link)
    double z_max = 1.0; // Valor máximo en coordenada Z (base_link)

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud_in);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(z_min, z_max);
    pass.filter(*cloud_out);
}
void pointCloudCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg) {
    // Transforma la nube de puntos al marco base_link
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (!transformPointCloud("base_link", msg, transformed_cloud)) {
        ROS_WARN("Failed to transform point cloud.");
        return;
    }

    // Filtra la nube de puntos en la coordenada Z de base_link
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    filterPointCloudZ(transformed_cloud, filtered_cloud);

    // Publica la nube de puntos filtrada
    pub.publish(filtered_cloud);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "point_cloud_filter_node");
    ros::NodeHandle nh;

    tf2_ros::TransformListener tf_listener(tf_buffer);
    // Suscribe al tópico de la nube de puntos de la cámara
    ros::Subscriber sub = nh.subscribe("/hsrb/head_rgbd_sensor/depth_registered/rectified_points", 1, pointCloudCallback);

    // Publica la nube de puntos filtrada
    pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("/filtered_point_cloud", 1);

    ros::spin();
    return 0;
}

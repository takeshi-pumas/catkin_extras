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
#include <point_cloud/PointCloudFilter.h>


ros::Publisher pub;
tf2_ros::Buffer tf_buffer;
//tf2_ros::TransformListener* tf_listener_ptr;


// Transform PC frame and filter PC by height functions
bool transformPointCloud(const std::string& target_frame, const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud_in,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out) {
    //static tf2_ros::TransformListener tf_listener(tf_buffer);
    ros::Time now = ros::Time(0); //::now();
    ros::Duration duration = ros::Duration(0.5);
    geometry_msgs::TransformStamped transformStamped;
    try {
        transformStamped = tf_buffer.lookupTransform(target_frame, cloud_in->header.frame_id, now, duration);
    } catch (tf2::TransformException& ex) {
        ROS_WARN("%s", ex.what());
        return false;
    }

    pcl_ros::transformPointCloud(target_frame, *cloud_in, *cloud_out, tf_buffer);
    return true;
}

void filterPointCloudZ(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud_in,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out, float min_height, float max_height) {
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud_in);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(min_height, max_height);
    pass.filter(*cloud_out);
}

// Filter PC service
bool filterPointCloudService(point_cloud::PointCloudFilter::Request& req,
                            point_cloud::PointCloudFilter::Response& res){
    
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr msg = ros::topic::waitForMessage<pcl::PointCloud<pcl::PointXYZ>>("/hsrb/head_rgbd_sensor/depth_registered/rectified_points");
    
    if (!msg) {
        ROS_WARN("Failed to receive point cloud message.");
        return false;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (!transformPointCloud("base_link", msg, transformed_cloud)) {
        ROS_WARN("Failed to transform point cloud.");
        return false;
    }

    // Filtra la nube de puntos en la coordenada Z de base_link
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    filterPointCloudZ(transformed_cloud, filtered_cloud, req.min_height, req.max_height);
    pcl::toROSMsg(*filtered_cloud, res.filtered_cloud);
    return true;
}

// Filter PC topic
void pointCloudCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg) {
    // Transforma la nube de puntos al marco base_link
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (!transformPointCloud("base_link", msg, transformed_cloud)) {
        ROS_WARN("Failed to transform point cloud.");
        return;
    }

    float z_min = 0.03;
    float z_max = 2.0;

    // Filtra la nube de puntos en la coordenada Z de base_link
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    filterPointCloudZ(transformed_cloud, filtered_cloud, z_min, z_max);

    // Publica la nube de puntos filtrada
    pub.publish(filtered_cloud);
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "point_cloud_filter_node");
    ros::NodeHandle nh;

    //tf_buffer.setUsingDedicatedThread(true);

    tf2_ros::TransformListener tf_listener(tf_buffer);
    //tf_listener_ptr = &tf_listener;
    // Suscribe al tópico de la nube de puntos de la cámara
    ros::Subscriber sub = nh.subscribe("/hsrb/head_rgbd_sensor/depth_registered/rectified_points", 1, pointCloudCallback);

    // Publica la nube de puntos filtrada
    pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("/filtered_point_cloud", 1);
    ros::ServiceServer service = nh.advertiseService("filter_point_cloud", filterPointCloudService);


    ros::spin();
    return 0;
}

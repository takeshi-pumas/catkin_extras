/*#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

class PointCloudTransformer {
public:
    PointCloudTransformer() : nh_("~") {
        tf_buffer_.reset(new tf2_ros::Buffer());
        tf_listener_.reset(new tf2_ros::TransformListener(*tf_buffer_));
        
        input_topic_ = "/hsrb/head_rgbd_sensor/depth_registered/rectified_points";
        output_topic_ = "/point_cloud_transformed";
        filter_threshold_ = 0.01;
        
        sub_ = nh_.subscribe(input_topic_, 1, &PointCloudTransformer::pointCloudCallback, this);
        pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_, 1);
        ROS_INFO("ROS PointCloud height filter");
        ROS_INFO("Height threshold: %f", filter_threshold_);
        ROS_INFO("Output_topic: %s", output_topic_.c_str());
        ROS_INFO("Service to change height: /to_be_defined");
    }

    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
        try {
            ROS_INFO("mensaje recibido");
            geometry_msgs::TransformStamped transform = tf_buffer_->lookupTransform("map", msg->header.frame_id, ros::Time(0), ros::Duration(1));
            ROS_INFO("Transform: %f, %f, %f", transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z);
            sensor_msgs::PointCloud2 transformed_cloud;
            tf2::doTransform(*msg, transformed_cloud, transform);

            sensor_msgs::PointCloud2Iterator<float> iter_x(transformed_cloud, "x");
            sensor_msgs::PointCloud2Iterator<float> iter_y(transformed_cloud, "y");
            sensor_msgs::PointCloud2Iterator<float> iter_z(transformed_cloud, "z");

            sensor_msgs::PointCloud2 filtered_cloud;
            filtered_cloud.header = transformed_cloud.header;

            for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
                //ROS_INFO("%f, %f, %f", iter_x, iter_y, iter_z);
                if (*iter_z >= filter_threshold_) {
                    *iter_x = *iter_x;
                    *iter_y = *iter_y;
                    *iter_z = *iter_z;
                }
            }

            pub_.publish(filtered_cloud);
        } catch (tf2::TransformException& ex) {
            ROS_WARN("Transform exception: %s", ex.what());
        }
    }

private:
    ros::NodeHandle nh_;
    std::string input_topic_;
    std::string output_topic_;
    double filter_threshold_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "point_cloud_transformer_node_cpp");
    PointCloudTransformer pc_transformer;
    ros::spin();
    return 0;
}*/

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

ros::Publisher pub;
ros::Subscriber sub;

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input_cloud) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

  // Convertimos de ROS a PCL
  pcl::fromROSMsg(*input_cloud, *cloud);

  // Filtramos la nube de puntos
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.0, 1.0);
  // pass.setNegative(true);  // Si deseas filtrar los puntos que NO estén dentro de los límites
  pass.filter(*cloud_filtered);

  // Convertimos de PCL a ROS
  sensor_msgs::PointCloud2 output_cloud;
  pcl::toROSMsg(*cloud_filtered, output_cloud);
  output_cloud.header = input_cloud->header;

  // Publicamos la nube de puntos filtrada
  pub.publish(output_cloud);
}

int main(int argc, char** argv) {
  // Inicializamos el nodo de ROS
  ros::init(argc, argv, "pcl_passthrough_filter_node");
  ros::NodeHandle nh;

  // Creamos un suscriptor para la nube de puntos
  sub = nh.subscribe<sensor_msgs::PointCloud2>("/hsrb/head_rgbd_sensor/depth_registered/rectified_points", 1, cloudCallback);

  // Creamos un publicador para la nube de puntos filtrada
  pub = nh.advertise<sensor_msgs::PointCloud2>("/point_cloud_transformed", 1);

  // Esperamos mensajes
  ros::spin();

  return 0;
}

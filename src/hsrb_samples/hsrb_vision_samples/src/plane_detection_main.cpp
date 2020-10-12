/// @brief Plane Detection Sample
/// @brief Copyright (C) 2016 Toyota Motor Corporation
#include <hsrb_vision_samples/plane_detection.hpp>

int main(int argc, char** argv) {
  ros::init(argc, argv, "plane_detection");
  hsrb_vision_samples::PlaneDetection plane_detection;
  ros::Rate spin_rate(10);

  pcl::visualization::CloudViewer plane_viewer("Plane Detection Viewer");

  while (ros::ok()) {
    ros::spinOnce();
    if (!plane_viewer.wasStopped()) {
      plane_viewer.showCloud(plane_detection.GetOutputCloud());
    }
    spin_rate.sleep();
  }

  return 0;
}

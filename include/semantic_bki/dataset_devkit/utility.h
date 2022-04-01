#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>

// Load pc predictions here
struct PointXYZProbs {
  pcl::PointCloud<pcl::PointXYZ> pc;
  std::vector<float> probs;

  PointXYZProbs(int nc) {
    probs.resize(nc)
  }
};
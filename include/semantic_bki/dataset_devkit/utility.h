#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>

// Load pc predictions here
struct PointXYZProbs {
  pcl::PointCloud<pcl::PointXYZ> pc;
  std::vector<std::vector<float> >probs;
  int frame_id;

  PointXYZProbs(int point_cnt, int nc) {
    probs.resize(point_cnt, std::vector<float>(nc, 0.0));
  }
};
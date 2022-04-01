#include <string>
#include <iostream>
#include <ros/ros.h>

#include "bkioctomap.h"
#include "markerarray_pub.h"
#include "semantickitti_util.h"
#include "utility.h"

// For ROS listener
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Bool.h"
#include <thread>

#include <memory>
#include <queue>

#define NUM_CLASSES       (int) 11

std::queue<std::shared_ptr<PointXYZProbs> > scans_q;

void prediction_scan_listener(std_msgs::Float32MultiArray scan_msg)
{
  const int frame_header_sz = 3;
  const int xyz_coord_sz    = 3;

  std::cout << "Received data of index: " << scan_msg.data.at(0) << std::endl;
  std::shared_ptr<PointXYZProbs> scan_ptr = \
    std::shared_ptr<PointXYZProbs>(new PointXYZProbs(NUM_CLASSES));
  // TODO: FIX PROBS 2D VECTOR!!
  int frame_num   = int(scan_msg.data.at(0));
  int num_points  = int(scan_msg.data.at(1));
  int num_classes = int(scan_msg.data.at(2));

  scan_ptr->frame_id = frame_num;

  int packet_sz = xyz_coord_sz + num_classes; // Size of each data packaet
  for(int packet_num = 0; packet_num < num_points; packet_num++)
  {
    pcl::PointXYZ point;
    int packet_curr_idx  = (packet_num * packet_sz) + frame_header_sz;

    // Add point xyz coords
    point.x = scan_msg.data.at(packet_curr_idx);
    point.y = scan_msg.data.at(packet_curr_idx+1);
    point.z = scan_msg.data.at(packet_curr_idx+2);
    scan_ptr->pc.push_back(point);

    // Add pred probs for each class
    for (int class_idx = 0; class_idx < num_classes; ++class_idx) {
      int prob_idx = packet_curr_idx + xyz_coord_sz + class_idx;
      scan_ptr->probs[packet_num][class_idx] = scan_msg.data.at(prob_idx);
    }
  }

  // Add scan to queue
  scans_q.push(scan_ptr);
}

void threaded_listener_function()
{
  int argc;
  char **argv;
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros:: Subscriber sub = n.subscribe("floats", 10, prediction_scan_listener);
  ros::spin();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "semantickitti_node");
    ros::NodeHandle nh("~");

    std::string map_topic("occupied_cells_vis_array");
    int block_depth = 4;
    double sf2 = 1.0;
    double ell = 1.0;
    float prior = 1.0f;
    float var_thresh = 1.0f;
    double free_thresh = 0.3;
    double occupied_thresh = 0.7;
    double resolution = 0.1;
    int num_class = 11;
    double free_resolution = 0.5;
    double ds_resolution = 0.1;
    int scan_num = 0;
    double max_range = -1;
    
    // SemanticKITTI
    std::string dir;
    std::string input_data_prefix;
    std::string input_label_prefix;
    std::string lidar_pose_file;
    std::string gt_label_prefix;
    std::string evaluation_result_prefix;
    bool query = false;
    bool visualize = false;

    nh.param<int>("block_depth", block_depth, block_depth);
    nh.param<double>("sf2", sf2, sf2);
    nh.param<double>("ell", ell, ell);
    nh.param<float>("prior", prior, prior);
    nh.param<float>("var_thresh", var_thresh, var_thresh);
    nh.param<double>("free_thresh", free_thresh, free_thresh);
    nh.param<double>("occupied_thresh", occupied_thresh, occupied_thresh);
    nh.param<double>("resolution", resolution, resolution);
    nh.param<int>("num_class", num_class, num_class);
    nh.param<double>("free_resolution", free_resolution, free_resolution);
    nh.param<double>("ds_resolution", ds_resolution, ds_resolution);
    nh.param<int>("scan_num", scan_num, scan_num);
    nh.param<double>("max_range", max_range, max_range);

    // SemanticKITTI
    nh.param<std::string>("dir", dir, dir);
    nh.param<std::string>("input_data_prefix", input_data_prefix, input_data_prefix);
    nh.param<std::string>("input_label_prefix", input_label_prefix, input_label_prefix);
    nh.param<std::string>("lidar_pose_file", lidar_pose_file, lidar_pose_file);
    nh.param<std::string>("gt_label_prefix", gt_label_prefix, gt_label_prefix);
    nh.param<std::string>("evaluation_result_prefix", evaluation_result_prefix, evaluation_result_prefix);
    nh.param<bool>("query", query, query);
    nh.param<bool>("visualize", visualize, visualize);

    ROS_INFO_STREAM("Parameters:" << std::endl <<
      "block_depth: " << block_depth << std::endl <<
      "sf2: " << sf2 << std::endl <<
      "ell: " << ell << std::endl <<
      "prior:" << prior << std::endl <<
      "var_thresh: " << var_thresh << std::endl <<
      "free_thresh: " << free_thresh << std::endl <<
      "occupied_thresh: " << occupied_thresh << std::endl <<
      "resolution: " << resolution << std::endl <<
      "num_class: " << num_class << std::endl << 
      "free_resolution: " << free_resolution << std::endl <<
      "ds_resolution: " << ds_resolution << std::endl <<
      "scan_num: " << scan_num << std::endl <<
      "max_range: " << max_range << std::endl <<

      "SemanticKITTI:" << std::endl <<
      "dir: " << dir << std::endl <<
      "input_data_prefix: " << input_data_prefix << std::endl <<
      "input_label_prefix: " << input_label_prefix << std::endl <<
      "lidar_pose_file: " << lidar_pose_file << std::endl <<
      "gt_label_prefix: " << gt_label_prefix << std::endl <<
      "evaluation_result_prefix: " << evaluation_result_prefix << std::endl <<
      "query: " << query << std::endl <<
      "visualize:" << visualize
      );

    // Initialize ros listener thread
    std::thread t1(threaded_listener_function);

    ///////// Build Map /////////////////////
    SemanticKITTIData semantic_kitti_data(nh, resolution, block_depth, sf2, ell, num_class, free_thresh, occupied_thresh, var_thresh, ds_resolution, free_resolution, max_range, map_topic, prior);
    semantic_kitti_data.read_lidar_poses(dir + '/' + lidar_pose_file);
    semantic_kitti_data.set_up_evaluation(dir + '/' + gt_label_prefix, dir + '/' + evaluation_result_prefix);

    while (1) {
      if (!scans_q.empty()) {
        std::shared_ptr<PointXYZProbs> scan = scans_q.front();
        scans_q.pop();
        std::string input_data_dir = dir + '/' + input_data_prefix;
        semantic_kitti_data.process_scan(scan, input_data_dir, query, visualize);
        std::cout << "after while loop" << std::endl;
      }
      ros::spin();
    }
    return 0;
}

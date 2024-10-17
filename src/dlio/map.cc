/***********************************************************
 *                                                         *
 * Copyright (c)                                           *
 *                                                         *
 * The Verifiable & Control-Theoretic Robotics (VECTR) Lab *
 * University of California, Los Angeles                   *
 *                                                         *
 * Authors: Kenny J. Chen, Ryan Nemiroff, Brett T. Lopez   *
 * Contact: {kennyjchen, ryguyn, btlopez}@ucla.edu         *
 *                                                         *
 ***********************************************************/

#include "dlio/map.h"
#include <filesystem>

dlio::MapNode::MapNode() : Node("dlio_map_node") {
  
  // Retrieve parameters
  this->getParams();

  // ROS 2 Subscriber and Publisher
  this->keyframe_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "keyframes", 10, std::bind(&dlio::MapNode::callbackKeyframe, this, std::placeholders::_1));

  this->map_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("map", 100);

  // ROS 2 Service
  this->save_pcd_srv = this->create_service<direct_lidar_inertial_odometry::srv::SavePCD>(
      "save_pcd", std::bind(&dlio::MapNode::savePCD, this, std::placeholders::_1, std::placeholders::_2));

  // Create a point cloud for storing the map
  this->dlio_map = pcl::PointCloud<PointType>::Ptr (boost::make_shared<pcl::PointCloud<PointType>>());

  pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
}

dlio::MapNode::~MapNode() {}

void dlio::MapNode::getParams() {
  // Parameter handling in ROS 2
  this->declare_parameter<std::string>("dlio/odom/odom_frame", "odom");
  this->get_parameter("dlio/odom/odom_frame", this->odom_frame);

  this->declare_parameter<double>("dlio/map/sparse/leafSize", 0.5);
  this->get_parameter("dlio/map/sparse/leafSize", this->leaf_size_);

  // Get node namespace and remove leading character
  std::string ns = this->get_namespace();
  ns.erase(0, 1); // Remove leading "/"

  // Concatenate frame name strings
  this->odom_frame = ns + "/" + this->odom_frame;
}

void dlio::MapNode::start() {
  // This method can be used to initialize the node further if needed
}

void dlio::MapNode::callbackKeyframe(sensor_msgs::msg::PointCloud2::ConstSharedPtr keyframe) {
  // Convert scan to PCL format
  pcl::PointCloud<PointType>::Ptr keyframe_pcl =
    pcl::PointCloud<PointType>::Ptr (boost::make_shared<pcl::PointCloud<PointType>>());
  pcl::fromROSMsg(*keyframe, *keyframe_pcl);

  // Apply voxel filter
  this->voxelgrid.setLeafSize(this->leaf_size_, this->leaf_size_, this->leaf_size_);
  this->voxelgrid.setInputCloud(keyframe_pcl);
  this->voxelgrid.filter(*keyframe_pcl);

  // Save filtered keyframe to the map
  *this->dlio_map += *keyframe_pcl;

  // Publish the full map
  if (this->dlio_map->points.size() == this->dlio_map->width * this->dlio_map->height) {
    sensor_msgs::msg::PointCloud2 map_ros;
    pcl::toROSMsg(*this->dlio_map, map_ros);
    map_ros.header.stamp = this->now();
    map_ros.header.frame_id = this->odom_frame;
    this->map_pub->publish(map_ros);
  }
}

bool dlio::MapNode::savePCD(
    std::shared_ptr<direct_lidar_inertial_odometry::srv::SavePCD::Request> req,
    std::shared_ptr<direct_lidar_inertial_odometry::srv::SavePCD::Response> res) {

  pcl::PointCloud<PointType>::Ptr m =
    pcl::PointCloud<PointType>::Ptr (boost::make_shared<pcl::PointCloud<PointType>>(*this->dlio_map));

  float leaf_size = req->leaf_size;
  std::string p = req->save_path;

  if (!std::filesystem::is_directory(p)) {
    std::cout << "Could not find directory " << p << std::endl;
    res->success = false;
    return false;
  }

  std::cout << std::setprecision(2) << "Saving map to " << p + "/dlio_map.pcd"
            << " with leaf size " << leaf_size << "... ";
  std::cout.flush();

  // Apply voxelization to the map
  pcl::VoxelGrid<PointType> vg;
  vg.setLeafSize(leaf_size, leaf_size, leaf_size);
  vg.setInputCloud(m);
  vg.filter(*m);

  // Save the map to a PCD file
  int ret = pcl::io::savePCDFileBinary(p + "/dlio_map.pcd", *m);
  res->success = ret == 0;

  if (res->success) {
    std::cout << "done" << std::endl;
  } else {
    std::cout << "failed" << std::endl;
  }

  return res->success;
}

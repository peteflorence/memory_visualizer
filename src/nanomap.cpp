#include "nanomap.h"

#define num_nearest_neighbors 1

void NanoMap::setCameraInfo(double bin, double width, double height, Matrix3 K_camera_info) {
  if (bin < 1.0) {binning = 1.0;}
  else {binning = bin;}
  num_x_pixels = width / binning;
  num_y_pixels = height / binning;
  K = K_camera_info;
  K /= binning;
  K(2,2) = 1.0;
  return;
}

void NanoMap::BuildNewKDTree(pcl::PointCloud<pcl::PointXYZ>::Ptr const& cloud_new) {
  latest_depth_image.organized_cloud_ptr = cloud_new;
  latest_depth_image.kd_tree.InitializeNew(cloud_new);
}

std::vector<pcl::PointXYZ> NanoMap::FindNearestPointsNew(Vector3 const& robot_position) {
  latest_depth_image.kd_tree.SearchForNearest<num_nearest_neighbors>(robot_position[0], robot_position[1], robot_position[2]);
  return latest_depth_image.kd_tree.closest_pts;
}

void NanoMap::AddToMergedKDTree(pcl::PointCloud<pcl::PointXYZ>::Ptr const& cloud_new) {
  merged_depth_images.kd_tree.AddToKDTree(cloud_new);
}

std::vector<pcl::PointXYZ> NanoMap::FindNearestPointsMerged(Vector3 const& robot_position) {
  merged_depth_images.kd_tree.SearchForNearest<num_nearest_neighbors>(robot_position[0], robot_position[1], robot_position[2]);
  return merged_depth_images.kd_tree.closest_pts;
}
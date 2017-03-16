#include <Eigen/Dense>
#include "kd_tree.hpp"
#include "nanoflann.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


typedef double Scalar;
typedef Eigen::Matrix<Scalar, 3, 1> Vector3;
typedef Eigen::Matrix<Scalar, 3, 3> Matrix3;
typedef Eigen::Matrix<Scalar, 4, 4> Matrix4;

#include <math.h>
#include <chrono>
#include <algorithm> 

class NanoMap {
public:

  void setCameraInfo(double bin, double width, double height, Matrix3 K_camera_info);

  void BuildNewKDTree(pcl::PointCloud<pcl::PointXYZ>::Ptr const& cloud_new);
  std::vector<pcl::PointXYZ> FindNearestPointsNew(Vector3 const& robot_position);

  void AddToMergedKDTree(pcl::PointCloud<pcl::PointXYZ>::Ptr const& cloud_new);
  void ClearMergedKDTree();
  std::vector<pcl::PointXYZ> FindNearestPointsMerged(Vector3 const& robot_position);

  void UpdatePose(size_t id, Matrix4 pose, Vector3 pose_axis_aligned_linear_covariance);

  // bool IsBehind(Vector3 robot_position);
  // bool IsOutsideDeadBand(Vector3 robot_position);
  // double IsOutsideFOV(Vector3 robot_position);
  // double AddOutsideFOVPenalty(Vector3 robot_position, double probability_of_collision);
  

private:

  Matrix4 current_pose;
  DepthPose latest_depth_pose;

  struct DepthPose {
    size_t id;          // unique id, can be used to update pose information, must be monotonically ascending with time
    uint32_t time_sec;  // time stamp can also be used to update pose information
    uint32_t time_nsec;

    Matrix4 pose;
    Vector3 pose_axis_aligned_linear_covariance;

    pcl::PointCloud<pcl::PointXYZ>::Ptr organized_cloud_ptr;
    KDTree<double> kd_tree;
  };

  KDTree<double> merged_kd_tree;

  Vector3 sigma_depth_point;
  Matrix3 K;
  double binning;
  double num_x_pixels;
  double num_y_pixels;

  double p_collision_behind;
  double p_collision_left_right_fov;
  double p_collision_up_down_fov;
  double p_collision_occluded;
  double p_collision_beyond_fov;

};

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

  // methods for handling organized point clouds (nanomap_ros can provide conversion from depth image to organized point clouds)
  void setCameraInfo(double bin, double width, double height, Matrix3 K_camera_info);
  void BuildNewKDTree(pcl::PointCloud<pcl::PointXYZ>::Ptr const& cloud_new);

  void AddToMergedKDTree(pcl::PointCloud<pcl::PointXYZ>::Ptr const& cloud_new);
  void ClearMergedKDTree();

  // methods for handling pose transforms
  void updateTransform(size_t depth_image_from, size_t depth_image_to, Matrix4 transform, Vector3 axis_aligned_linear_covariance);

  // methods for handling approximate k-nearest-neighbor queries
  // k (num_nearest_neighbors) is #defined in nanomap.cpp
  std::vector<pcl::PointXYZ> FindNearestPointsReverseSearch(Vector3 const& robot_position, Vector3 bounding_box);
  std::vector<pcl::PointXYZ> FindNearestPointsMerged(Vector3 const& robot_position, Vector3 bounding_box);

private:

  struct DepthImage {
    size_t id;   // unique id, assigned to be monotonically increasing with time
    pcl::PointCloud<pcl::PointXYZ>::Ptr organized_cloud_ptr;
    KDTree<double> kd_tree;
  };

  struct PoseTransform {
    size_t depth_image_from;
    size_t depth_image_to;

    Matrix4 transform;
    Vector3 axis_aligned_linear_covariance;
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

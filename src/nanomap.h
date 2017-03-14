#include "kd_tree.h"
#include "nanoflann.hpp"

#include <Eigen/Dense>

typedef double Scalar;
typedef Eigen::Matrix<Scalar, 3, 1> Vector3;
typedef Eigen::Matrix<Scalar, 3, 3> Matrix3;

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <math.h>
#include <chrono>
#include <algorithm> 

class NanoMap {
public:

  void setCameraInfo(double bin, double width, double height, Matrix3 K_camera_info);

  void BuildNewKDTree(pcl::PointCloud<pcl::PointXYZ>::Ptr const& cloud_new);
  std::vector<pcl::PointXYZ> FindNearestPointsNew(Vector3 const& robot_position);

  void AddToMergedKDTree(pcl::PointCloud<pcl::PointXYZ>::Ptr const& cloud_new);
  std::vector<pcl::PointXYZ> FindNearestPointsMerged(Vector3 const& robot_position);

  // bool IsBehind(Vector3 robot_position);
  // bool IsOutsideDeadBand(Vector3 robot_position);
  // double IsOutsideFOV(Vector3 robot_position);
  // double AddOutsideFOVPenalty(Vector3 robot_position, double probability_of_collision);
  

private:

  struct OneDepthImageKDTree {
    pcl::PointCloud<pcl::PointXYZ>::Ptr organized_cloud_ptr;
    KDTree<double> kd_tree;
  };
  OneDepthImageKDTree latest_depth_image;

  struct MergedDepthImageKDTree {
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> organized_cloud_ptrs;
    KDTree<double> kd_tree;
  };
  MergedDepthImageKDTree merged_depth_images;


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

  class LinkedListKDTrees {
    struct Node {
      OneDepthImageKDTree const* kd_tree;
      Node *next;
      Node *previous;
    };

  private:
    Node *head;
    Node *tail;
    size_t num_nodes = 0;
    size_t max_nodes;

  public:
    LinkedListKDTrees() {
      head = NULL;
      tail = NULL;
    }

    void addNode(OneDepthImageKDTree const* new_kd_tree) {
      if (num_nodes < max_nodes) {
        Node *n = new Node();
        n->kd_tree = new_kd_tree;
        if (num_nodes==0) {
          tail = n;
        }
        if (num_nodes==1) {
          second_from_tail = n;
        }
        n->next = head;
        head = n;
        num_nodes++;
      }
      else {
        return;
      }
    }

    void setMaxKDTrees(size_t set_max) {
      max_nodes = set_max;
    }

  };

};

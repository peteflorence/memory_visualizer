/**
 * Copyright 2017 Massachusetts Institute of Technology
 *
 * @file pose_point_cloud_graph.h
 * @author Pete Florence
 */

#include "circular_buffer.h"

typedef double Scalar;
typedef Eigen::Matrix<Scalar, 3, 1> Vector3;
typedef Eigen::Matrix<Scalar, 3, 3> Matrix3;
typedef Eigen::Matrix<Scalar, 4, 4> Matrix4;

namespace nanomap {

struct PointCloudData {
  uint32_t id;   // unique id, assigned to be monotonically increasing with time
  pcl::PointCloud<pcl::PointXYZ>::Ptr organized_cloud_ptr;
  KDTree<double> kd_tree;
};

struct TransformData {
  Matrix4 transform;
  Vector3 axis_aligned_linear_covariance;
};

template <typename VertexData, typename EdgeData>
class GraphChain {

  GraphChain(uint32_t capacity) {
    capacity_ = capacity;
    edges_ = CircularBuffer<DirectedEdge>(capacity);
    vertices_ = CircularBuffer<Vertex>(capacity);
  }

  class Vertex;
  class DirectedEdge;

  class Vertex {
    VertexData vertex_data;
    DirectedEdge edge;
  };

  class DirectedEdge {
    EdgeData edge_data;
    Vertex vertex;
  };

  CircularBuffer<DirectedEdge> edges_;
  CircularBuffer<Vertex> vertices_;
  uint32_t capacity_;

};

} // namespace nanomap
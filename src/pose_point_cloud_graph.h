/**
 * Copyright 2017 Massachusetts Institute of Technology
 *
 * @file pose_point_cloud_graph.h
 * @author Pete Florence
 */

 #include "circular_buffer.h"

template <typename VertexData>
class Vertex {
  VertexData vertex_data;
  DirectedEdge edge;
};

template <typename EdgeData>
class DirectedEdge {
  EdgeData edge_data;
  Vertex vertex;
};

struct PointCloudData {
  size_t id;   // unique id, assigned to be monotonically increasing with time
  pcl::PointCloud<pcl::PointXYZ>::Ptr organized_cloud_ptr;
  KDTree<double> kd_tree;
};

struct TransformData {
  size_t depth_image_from;
  size_t depth_image_to;

  Matrix4 transform;
  Vector3 axis_aligned_linear_covariance;
};

class PosePointCloudChain {
  PosePointCloudChain(size_t capacity) {
    edges = CircularBuffer<DirectedEdge<TransformData>>(capacity);
    vertices = CircularBuffer<Vertex<PointCloudData>>(capacity);

    this->capacity = capacity;
  }

  CircularBuffer<Edge<PointCloudData>> edges;
  CircularBuffer<Vertex<TransformData>> vertices;
  size_t capacity;
};
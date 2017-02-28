#include "transform_utils.h"

Matrix3 constructR( geometry_msgs::PoseStamped const& pose ) {
  Eigen::Quaternion<Scalar> quat(pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z);
  return quat.toRotationMatrix();
}

Vector3 constructt( geometry_msgs::PoseStamped const& pose ) {
  return Vector3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
}

Eigen::Matrix3f constructRf( geometry_msgs::PoseStamped const& pose ) {
  Eigen::Quaternionf quat(pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z);
  return quat.toRotationMatrix();
}

Eigen::Vector3f constructtf( geometry_msgs::PoseStamped const& pose ) {
  return Eigen::Vector3f(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
}


Eigen::Matrix4d findTransform(geometry_msgs::PoseStamped const& new_pose, geometry_msgs::PoseStamped const& previous_pose) {
  return findTransform(previous_pose)*invertTransform(findTransform(new_pose));
}

Eigen::Matrix4d findTransform(geometry_msgs::PoseStamped const& pose) {
  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
  Matrix3 R = constructR(pose);
  Vector3 t = constructt(pose);
  transform.block<3,3>(0,0) = R;
  transform.block<3,1>(0,3) = t;
  return transform;
}

Eigen::Matrix4f findTransform4f(geometry_msgs::PoseStamped const& pose) {
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  Eigen::Matrix3f R = constructRf(pose);
  Eigen::Vector3f t = constructtf(pose);
  transform.block<3,3>(0,0) = R;
  transform.block<3,1>(0,3) = t;
  return transform;
}

Eigen::Matrix4d invertTransform(Eigen::Matrix4d transform) {
  Matrix3 R = transform.block<3,3>(0,0);
  Vector3 t = transform.block<3,1>(0,3);
  Eigen::Matrix4d inverted_transform = Eigen::Matrix4d::Identity();
  inverted_transform.block<3,3>(0,0) = R.transpose();
  inverted_transform.block<3,1>(0,3) = -1.0 * R.transpose() * t;
  return inverted_transform;
}

Vector3 applyTransform(Vector3 p, Eigen::Matrix4d transform) {
  Vector4 p_aug;
  p_aug << p, 1.0;
  p_aug = transform * p_aug;
  return Vector3(p_aug(0), p_aug(1), p_aug(2));
}
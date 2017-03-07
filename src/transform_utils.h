#ifndef TRANSFORM_UTILS_H
#define TRANSFORM_UTILS_H

#include <Eigen/Dense>
typedef double Scalar;
typedef Eigen::Matrix<Scalar, 3, 1> Vector3;
typedef Eigen::Matrix<Scalar, 4, 1> Vector4;
typedef Eigen::Matrix<Scalar, 3, 3> Matrix3;
#include "geometry_msgs/PoseStamped.h"

Matrix3 constructR( geometry_msgs::PoseStamped const& pose );

Vector3 constructt( geometry_msgs::PoseStamped const& pose );

Eigen::Matrix3f constructRf( geometry_msgs::PoseStamped const& pose );

Eigen::Vector3f constructtf( geometry_msgs::PoseStamped const& pose );

Eigen::Matrix4d findTransform(geometry_msgs::PoseStamped const& new_pose, geometry_msgs::PoseStamped const& previous_pose);

Eigen::Matrix4d findTransform(geometry_msgs::PoseStamped const& pose);

Eigen::Matrix4f findTransform4f(geometry_msgs::PoseStamped const& pose);

Eigen::Matrix4d invertTransform(Eigen::Matrix4d transform);

Vector3 applyTransform(Vector3 p, Eigen::Matrix4d transform);

#endif
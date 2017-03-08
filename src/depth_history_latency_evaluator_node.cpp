#include "transform_utils.h"
#include "fov_visualizer.h"
#include "fov_evaluator.h"
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Point.h"
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include "pcl_ros/transforms.h"
#include "pcl_ros/impl/transforms.hpp"

#include "tf/tf.h"
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <cmath>
#include <stdlib.h>
#include <chrono>


class DepthHistoryLatencyEvaluatorNode {
public:

	DepthHistoryLatencyEvaluatorNode() {
		// Subscribers
		pose_sub = nh.subscribe("/pose", 100, &DepthHistoryLatencyEvaluatorNode::OnPose, this);
    smoothed_path_sub = nh.subscribe("/samros/keyposes", 100, &DepthHistoryLatencyEvaluatorNode::OnSmoothedPath, this);

    camera_info_sub = nh.subscribe("depth_camera_info", 1, &DepthHistoryLatencyEvaluatorNode::OnCameraInfo, this);
    depth_image_sub = nh.subscribe("depth_camera_pointcloud", 100, &DepthHistoryLatencyEvaluatorNode::OnDepthImage, this);

		tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);

    srand(time(NULL)); // initialize random seed

    for(;;){
        try {

          tf_buffer_.lookupTransform("world", "body", 
                                        ros::Time(0), ros::Duration(30.0));
        } catch (tf2::TransformException &ex) {
          continue;
        }

        break;
    }

    std::cout << "initiated" << std::endl;
	}

private:

  std::vector<geometry_msgs::PoseStamped> poses_path;
  std::vector<sensor_msgs::PointCloud2ConstPtr> point_cloud_ptrs;

  Matrix3 BodyToRDF;
  Matrix3 BodyToRDF_inverse;
  Eigen::Matrix4d transform_body_to_world; // better if not class variable


  bool got_camera_info = false;
  std::string depth_sensor_frame = "depth_sensor";
  void OnCameraInfo(const sensor_msgs::CameraInfo msg) {
    if (got_camera_info) {
      return;
    }
    got_camera_info = true;
    depth_sensor_frame = msg.header.frame_id;
  }

  Eigen::Matrix4f GetRDFToBodyTransform() {
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    Eigen::Matrix3f R = BodyToRDF_inverse.cast <float> ();
    transform.block<3,3>(0,0) = R;
    return transform;
  }

  size_t point_cloud_ctr = 0;
  void OnDepthImage(const sensor_msgs::PointCloud2ConstPtr& point_cloud_msg) {
    //ROS_INFO("GOT POINT CLOUD");
    size_t num_point_clouds = 90;
    if (point_cloud_ptrs.size() < num_point_clouds) {
      point_cloud_ptrs.push_back(point_cloud_msg);
      return;
    }
    point_cloud_ctr++;
  }


  void findTransformFromSmoothedPath(ros::Time query_time, Eigen::Matrix4f &transform, bool &can_interpolate) {
    if (last_smoothed_path.poses.size() > 0) {
      if ((query_time < last_smoothed_path.poses[0].header.stamp) || (query_time > last_smoothed_path.poses[last_smoothed_path.poses.size()-1].header.stamp)) {
        can_interpolate = false;
        return;
      }
      can_interpolate = true;

      // find pose_before and pose_after
      geometry_msgs::PoseStamped pose_before = last_smoothed_path.poses[0];
      geometry_msgs::PoseStamped pose_after;
      for (size_t i = 1; i < last_smoothed_path.poses.size(); i++) {
        pose_after = last_smoothed_path.poses[i];
        if (pose_after.header.stamp > query_time) {
          break;
        }
        pose_before = pose_after;
      }

      // find pose interpolation parameter
      double t_1 = query_time.toSec() - pose_before.header.stamp.toSec();
      double t_2 = pose_after.header.stamp.toSec() - query_time.toSec();

      //ros::Duration t_1_time = query_time - pose_before.header.stamp; // if need a few more digits of precision, can switch
      //ros::Duration t_2_time = pose_after.header.stamp - query_time;
      // std::cout << "t_1 " << t_1 << std::endl;
      // std::cout << "t_2 " << t_2 << std::endl;
      // std::cout << "t_1_time " << t_1_time << std::endl;
      // std::cout << "t_2_time " << t_2_time << std::endl;

      double t_parameter = t_1 / (t_1 + t_2);
      transform = InterpolateBetweenPoses(pose_before, pose_after, t_parameter, query_time);
    }
  }

  Eigen::Matrix4f InterpolateBetweenPoses(geometry_msgs::PoseStamped const& pose_before, geometry_msgs::PoseStamped const& pose_after, double t_parameter, ros::Time time_for_pose) {

    // need to actually do interpolation

    // position interpolation
    Eigen::Vector3f vector_before(pose_before.pose.position.x, pose_before.pose.position.y, pose_before.pose.position.z);
    Eigen::Vector3f vector_after(pose_after.pose.position.x, pose_after.pose.position.y, pose_after.pose.position.z);
    Eigen::Vector3f interpolated_vector = vector_before + (vector_after - vector_before)*t_parameter;

    Eigen::Quaternionf quat_before(pose_before.pose.orientation.w, pose_before.pose.orientation.x, pose_before.pose.orientation.y, pose_before.pose.orientation.z);
    Eigen::Quaternionf quat_after(pose_after.pose.orientation.w, pose_after.pose.orientation.x, pose_after.pose.orientation.y, pose_after.pose.orientation.z);
    Eigen::Quaternionf interpolated_quat;
    interpolated_quat = quat_before.slerp(t_parameter, quat_after);

    Eigen::Matrix4f interpolated_transform = Eigen::Matrix4f::Identity();
    interpolated_transform.block<3,3>(0,0) = interpolated_quat.toRotationMatrix();;
    interpolated_transform.block<3,1>(0,3) = interpolated_vector;
    return interpolated_transform;
  }


 void TransformToWorldFromPose(Eigen::Matrix4f transform_to_world, const sensor_msgs::PointCloud2ConstPtr msg, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out){
    sensor_msgs::PointCloud2 msg_out;
    msg_out.header.frame_id = "world";

    pcl_ros::transformPointCloud(transform_to_world, *msg, msg_out);

    pcl::PCLPointCloud2 cloud2;
    pcl_conversions::toPCL(msg_out, cloud2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(cloud2,*cloud);

    cloud_out = cloud;
  }



  size_t pose_ctr = 0;
  void OnPose( geometry_msgs::PoseStamped const& pose ) {
    //std::cout << "got pose: " << pose_ctr << std::endl;
    pose_ctr++;
    
  }

  nav_msgs::Path last_smoothed_path;
  void OnSmoothedPath( nav_msgs::Path const& path_msg ) {
    ROS_INFO("GOT SMOOTHED PATH");
    last_smoothed_path = path_msg;
  }


  Matrix3 GetBodyToRDFRotationMatrix() {
    geometry_msgs::TransformStamped tf;
      try {
        tf = tf_buffer_.lookupTransform(depth_sensor_frame, "body", 
                                    ros::Time(0), ros::Duration(1/30.0));
      } catch (tf2::TransformException &ex) {
        ROS_ERROR("%s", ex.what());
        return Matrix3();
      }
      Eigen::Quaternion<Scalar> quat(tf.transform.rotation.w, tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z);
      Matrix3 R = quat.toRotationMatrix();
      return R;
  }

	geometry_msgs::PoseStamped PoseFromVector3(Vector3 const& position, std::string const& frame) {
		geometry_msgs::PoseStamped pose;
		pose.pose.position.x = position(0);
		pose.pose.position.y = position(1);
		pose.pose.position.z = position(2);
		pose.header.frame_id = frame;
		pose.header.stamp = ros::Time::now();
		return pose;
	}

	Vector3 VectorFromPose(geometry_msgs::PoseStamped const& pose) {
		return Vector3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
	}

	std::string drawing_frame = "world";

	ros::Subscriber pose_sub;
  ros::Subscriber smoothed_path_sub;
  ros::Subscriber camera_info_sub;
  ros::Subscriber depth_image_sub;

	std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
	tf2_ros::Buffer tf_buffer_;

	ros::NodeHandle nh;
};


int main(int argc, char* argv[]) {
	std::cout << "Initializing memory_visualizer_node node" << std::endl;
	ros::init(argc, argv, "DepthHistoryLatencyEvaluator");
	DepthHistoryLatencyEvaluatorNode depth_history_latency_evaluator_node;
  ros::Rate spin_rate(100);

  while (ros::ok()) {
    //std::cout << "spin" << std::endl;
    ros::spinOnce();
    spin_rate.sleep();
  }

}

#include "transform_utils.h"
#include "fov_visualizer.h"
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Point.h"

#include "tf/tf.h"
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <cmath>
#include <stdlib.h>
#include <chrono>


class MemoryVisualizerNode {
public:

	MemoryVisualizerNode() {
		// Subscribers
		pose_sub = nh.subscribe("/pose", 1, &MemoryVisualizerNode::OnPose, this);
  	    // Publishers
		fov_pub = nh.advertise<visualization_msgs::Marker>("fov", 0);
		tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);
	}

private:

  bool initiated = false;
	int counter = 0; // this just reduces to 33 Hz from 100 Hz
  std::vector<Eigen::Matrix4d> odometries;

  geometry_msgs::PoseStamped last_pose;
  Matrix3 BodyToRDF;
  Matrix3 BodyToRDF_inverse;
  Eigen::Matrix4d transform_body_to_world; // better if not class variable

  void OnPose( geometry_msgs::PoseStamped const& pose ) {
    if (!initiated) {
      last_pose = pose;
      Eigen::Matrix4d transform_identity = Eigen::Matrix4d::Identity();
      transform_identity(0,3) = -1.0;
      std::cout << transform_identity << std::endl;
      for (int i = 0; i < 50; i++) {
        odometries.push_back(transform_identity);
      }
      initiated = true;
      return;
    }
    
    counter++;
    if (counter >= 10) {
      BodyToRDF = GetBodyToRDFRotationMatrix(); // could do only once later -- being safe for now
      BodyToRDF_inverse = BodyToRDF.inverse();
      counter = 0;
      ROS_INFO("GOT POSE");

      AddToOdometries(findTransform(pose, last_pose));
      transform_body_to_world = findTransform(pose);
      last_pose = pose;

      PublishFovMarkers();
      DeBug();
    }
  }

  void AddToOdometries(Eigen::Matrix4d current_transform) {
    std::cout << "rotate and add" << std::endl;
    rotate(odometries.begin(),odometries.end()-1,odometries.end()); // Shift vector so each move back 1
    odometries.at(0) = current_transform;
  }

  void DeBug() {
    Vector3 world_position(0,0,1);
    Eigen::Matrix4d transform_to_previous_body_frame = invertTransform(transformFromPreviousBodyToWorld(10));

    
    Vector3 previous_body_frame_position = applyTransform(world_position, transform_to_previous_body_frame);
    Eigen::Matrix4d transform_to_world = transformFromPreviousBodyToWorld(10);
    Vector3 new_world_position = applyTransform(previous_body_frame_position, transform_to_world);
    PublishPositionMarker(new_world_position);
  }

  void PublishPositionMarker(Vector3 position_world_frame) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = drawing_frame;
    marker.header.stamp = ros::Time::now();
    marker.ns = "fov_center";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = position_world_frame(0);
    marker.pose.position.y = position_world_frame(1);
    marker.pose.position.z = position_world_frame(2);
    marker.scale.x = 0.8;
    marker.scale.y = 0.8;
    marker.scale.z = 0.8;
    marker.color.a = 0.40; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    fov_pub.publish( marker );
  } 

  // this function works
  Eigen::Matrix4d transformFromPreviousBodyToWorld(int fov_id) {
    Eigen::Matrix4d transform = transform_body_to_world;
    for (int i = 0; i < fov_id; i++) {
      transform = odometries.at(i) * transform;
    }
    return transform;
  }

  // this function under construction
  Eigen::Matrix4d transformFromCurrentBodyToPreviousBody(int fov_id) {
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    for (int i = 0; i < fov_id; i++) {
      transform = transform * invertTransform(odometries.at(i));
    }
    return transform;
  }

  void PublishFovMarkers() {
    bool color_in_fov = false;
    for (int fov_id = 0; fov_id < odometries.size(); fov_id++) {
      // FIND CORNERS

      // start in that poses' rdf, and rotate to body
      Vector3 bottom_right = BodyToRDF_inverse * Vector3(7,5.25,10);
      Vector3 top_right = BodyToRDF_inverse * Vector3(7,-5.25,10);
      Vector3 top_left = BodyToRDF_inverse * Vector3(-7,-5.25,10);
      Vector3 bottom_left = BodyToRDF_inverse * Vector3(-7,5.25,10);
      Vector3 body = Vector3(0,0,0);

      Eigen::Matrix4d transform = transformFromPreviousBodyToWorld(fov_id);
      body = applyTransform(body, transform); // don't need to rotate 0,0,0
      Vector3 corner_1 = applyTransform(bottom_right, transform);
      Vector3 corner_2 = applyTransform(top_right, transform);
      Vector3 corner_3 = applyTransform(top_left, transform);
      Vector3 corner_4 = applyTransform(bottom_left, transform);


      // DETERMINE IF -1 in current body frame is in front of previous pose
      Vector3 position_current_body_frame(-1.0,0.0,0.0);
      Eigen::Matrix4d transform_2 = transformFromCurrentBodyToPreviousBody(fov_id);
      Vector3 position_previous_body_frame = applyTransform(position_previous_body_frame, transform_2);
      if (position_previous_body_frame(0) > 0) {
        color_in_fov = true;
      } 
      else {
        color_in_fov = false;
      }
      PublishFovMarker(fov_id, body, corner_1, corner_2, corner_3, corner_4, color_in_fov);
   }
  
   Eigen::Matrix4d transform = transformFromPreviousBodyToWorld(0);
   Vector3 position_current_body_frame(-1.0,0.0,0.0);
   Vector3 position_world_frame = applyTransform(position_current_body_frame, transform);
   //PublishPositionMarker(position_world_frame);
 }

	void PublishFovMarker(int fov_id, Vector3 body, Vector3 corner_1, Vector3 corner_2, Vector3 corner_3, Vector3 corner_4, bool color_in_fov) {
		std::vector<visualization_msgs::Marker> markers = BuildFovMarker(fov_id, body, corner_1, corner_2, corner_3, corner_4, color_in_fov);
   	fov_pub.publish( markers.at(0) );
    fov_pub.publish( markers.at(1) );
	}

  Matrix3 GetBodyToRDFRotationMatrix() {
    geometry_msgs::TransformStamped tf;
      try {
        tf = tf_buffer_.lookupTransform("r200_depth_optical_frame", "body", 
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

	ros::Publisher fov_pub;

	std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
	tf2_ros::Buffer tf_buffer_;

	ros::NodeHandle nh;
};


int main(int argc, char* argv[]) {
	std::cout << "Initializing memory_visualizer_node node" << std::endl;
	ros::init(argc, argv, "MemoryVisualizerNode");
	MemoryVisualizerNode memory_visualizer_node;
	ros::spin();
}

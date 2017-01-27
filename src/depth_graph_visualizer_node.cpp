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

      PublishPositionMarkers();
      PublishFovMarkerSimple(3);
      //PublishFovMarkers();
    }
  }

  void AddToOdometries(Eigen::Matrix4d current_transform) {
    std::cout << "rotate and add" << std::endl;
    rotate(odometries.begin(),odometries.end()-1,odometries.end()); // Shift vector so each move back 1
    odometries.at(0) = current_transform;
  }

  void PublishPositionMarkers()  {
    for (int fov_id = 0; fov_id < odometries.size(); fov_id++) {
      PublishPositionMarker(fov_id);
    }
  }

  void PublishPositionMarker(int fov_id) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = drawing_frame;
    marker.header.stamp = ros::Time::now();
    marker.ns = "fov_center";
    marker.id = fov_id;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;

    // this worked
    // Vector3 current_position_world_frame = VectorFromPose(last_pose);
    // Vector3 position_world_frame = transformFromCurrentPoseThroughChain(current_position_world_frame, fov_id); 

    Vector3 position_previous_body_frame(0,0,0);
    Eigen::Matrix4d transform = transformFromPreviousBodyToWorld(fov_id);
    Vector3 position_world_frame = applyTransform(position_previous_body_frame, transform);

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

  void PublishFovMarkerSimple(int fov_id) {   
    visualization_msgs::Marker marker;    
    marker.header.frame_id = drawing_frame;   
    marker.header.stamp = ros::Time::now();   
    marker.ns = "fov_simple";   
    marker.id = fov_id+100;   
    marker.type = visualization_msgs::Marker::SPHERE;   
    marker.action = visualization_msgs::Marker::ADD;    
         
    // // start in current rdf frame    
    Vector3 p = Vector3(0.0,0.0,1.0);   
    // // rotate to current body frame    
    p = BodyToRDF_inverse * p;    
    // put into world   

    for (int i = fov_id-1; i>=0; i--) {
      std::cout << "i " << i << std::endl;
      p = applyTransform(p, Eigen::Matrix4d::Identity());
    }

    transform_body_to_world = findTransform(last_pose);   
    p = applyTransform(p, transform_body_to_world);   
     
    marker.pose.position.x = p(0);    
    marker.pose.position.y = p(1);    
    marker.pose.position.z = p(2);    
    marker.scale.x = 0.8;   
    marker.scale.y = 0.8;   
    marker.scale.z = 0.8;   
    marker.color.a = 0.40; // Don't forget to set the alpha!    
    marker.color.r = 1.0;   
    marker.color.g = 1.0;   
    marker.color.b = 0.0;   
    fov_pub.publish( marker );     
   }

  Vector3 transformFromCurrentPoseThroughChain(Vector3 current_position, int fov_id) {
    for (int i = 0; i < fov_id; i++) {
      current_position = applyTransform(current_position, odometries.at(i));
    }
    return current_position;
  }

  Eigen::Matrix4d transformFromPreviousBodyToWorld(int fov_id) {
    Eigen::Matrix4d transform = transform_body_to_world;
    for (int i = 0; i < fov_id; i++) {
      transform = odometries.at(i) * transform;
    }
    return transform;
  }

  void PublishFovMarkers() {
    // // this works for one
    // Vector3 bottom_right = BodyToRDF_inverse * Vector3(7,5.25,10);
    // Vector3 top_right = BodyToRDF_inverse * Vector3(7,-5.25,10);
    // Vector3 top_left = BodyToRDF_inverse * Vector3(-7,-5.25,10);
    // Vector3 bottom_left = BodyToRDF_inverse * Vector3(-7,5.25,10);
    // Vector3 body = applyTransform(Vector3(0,0,0), transform_body_to_world); // don't need to rotate 0,0,0

    // Vector3 corner_1 = applyTransform(bottom_right, transform_body_to_world);
    // Vector3 corner_2 = applyTransform(top_right, transform_body_to_world);
    // Vector3 corner_3 = applyTransform(top_left, transform_body_to_world);
    // Vector3 corner_4 = applyTransform(bottom_left, transform_body_to_world);

    // PublishFovMarker(0, body, corner_1, corner_2, corner_3, corner_4);

    for (int fov_id = 0; fov_id < odometries.size(); fov_id++) {
      // start in that poses' rdf, and rotate to body
      Vector3 bottom_right = BodyToRDF_inverse * Vector3(7,5.25,10);
      Vector3 top_right = BodyToRDF_inverse * Vector3(7,-5.25,10);
      Vector3 top_left = BodyToRDF_inverse * Vector3(-7,-5.25,10);
      Vector3 bottom_left = BodyToRDF_inverse * Vector3(-7,5.25,10);
      Vector3 body = Vector3(0,0,0);

      for (int i = 0; i < fov_id; i++) {
        // convert through chain
        Eigen::Matrix4d transform = odometries.at(i);
        bottom_right = applyTransform(bottom_right, transform);
        top_right = applyTransform(top_right, transform);
        top_left = applyTransform(top_left, transform);
        bottom_left = applyTransform(bottom_left, transform);
        body = applyTransform(body, transform);
      }

    body = applyTransform(body, transform_body_to_world); // don't need to rotate 0,0,0
    Vector3 corner_1 = applyTransform(bottom_right, transform_body_to_world);
    Vector3 corner_2 = applyTransform(top_right, transform_body_to_world);
    Vector3 corner_3 = applyTransform(top_left, transform_body_to_world);
    Vector3 corner_4 = applyTransform(bottom_left, transform_body_to_world);
    PublishFovMarker(fov_id, body, corner_1, corner_2, corner_3, corner_4);
   }
 }


	void PublishFovMarker(int fov_id, Vector3 body, Vector3 corner_1, Vector3 corner_2, Vector3 corner_3, Vector3 corner_4) {
		std::vector<visualization_msgs::Marker> markers = BuildFovMarker(fov_id, body, corner_1, corner_2, corner_3, corner_4);
   	fov_pub.publish( markers.at(0) );
    fov_pub.publish( markers.at(1) );
	}

  Vector3 transformToCurrentRDFframe(Vector3 position_other_rdf_frame, int fov_id) {
    Eigen::Matrix4d transform_2 = Eigen::Matrix4d::Identity();

    Vector4 position_current_rdf_frame;
    position_current_rdf_frame << position_other_rdf_frame(0), position_other_rdf_frame(1), position_other_rdf_frame(2), 1.0;

    for (int i = 0; i < fov_id; i++) {
      position_current_rdf_frame = odometries.at(i) * position_current_rdf_frame;
    }

    Vector3 answer;
    answer << position_current_rdf_frame(0), position_current_rdf_frame(1), position_current_rdf_frame(2);
    return answer;
  }

  geometry_msgs::PoseStamped TransfromWorldPoseToRDFPose(geometry_msgs::PoseStamped pose_world_frame) {
      geometry_msgs::TransformStamped tf;
      try {
        tf = tf_buffer_.lookupTransform("r200_depth_optical_frame", "world",
                                      ros::Time(0), ros::Duration(1/30.0));
      } catch (tf2::TransformException &ex) {
        ROS_ERROR("ID 7 %s", ex.what());
        return PoseFromVector3(Vector3(0,0,0), "r200_depth_optical_frame");
      }
      geometry_msgs::PoseStamped pose_rdf_frame = PoseFromVector3(Vector3(0,0,0), "r200_depth_optical_frame");
      tf2::doTransform(pose_world_frame, pose_rdf_frame, tf);
      return pose_rdf_frame;
  }


	Vector3 TransformRDFtoWorld(Vector3 const& ortho_body_frame) {
		geometry_msgs::TransformStamped tf;
	    try {
	      tf = tf_buffer_.lookupTransform("world", "r200_depth_optical_frame",
	                                    ros::Time(0), ros::Duration(1/30.0));
	    } catch (tf2::TransformException &ex) {
	      ROS_ERROR("ID 7 %s", ex.what());
	      return Vector3(0,0,0);
	    }

	    geometry_msgs::PoseStamped pose_ortho_body_vector = PoseFromVector3(ortho_body_frame, "r200_depth_optical_frame");
    	geometry_msgs::PoseStamped pose_vector_world_frame = PoseFromVector3(Vector3(0,0,0), "world");
    	tf2::doTransform(pose_ortho_body_vector, pose_vector_world_frame, tf);
    	return VectorFromPose(pose_vector_world_frame);
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

  void PrintVectorPractice() {
    std::vector<int> v;
    int num = 6;
    for (int i = 0; i < num; i++) {
      v.push_back(i);
    } 

    std::cout << "Before rotating " << std::endl;
    for (int i = 0; i < num; i++) {
      std::cout << v.at(i) << std::endl;;
    } 

    std::cout << "After rotating " << std::endl;
    rotate(v.begin(),v.end()-1,v.end());
    for (int i = 0; i < num; i++) {
      std::cout << v.at(i) << std::endl;;
    } 

    std::cout << "New at top " << std::endl;
    v.at(0) = -1;
    for (int i = 0; i < num; i++) {
      std::cout << v.at(i) << std::endl;;
    } 

  }

  // void PrintToyTransform() {
  //   Vector3 p1, p2;
  //   p1 << 1,1,1;
  //   std::cout << "p1 " << p1 << std::endl;
  //   Eigen::Affine3d transform_2 = Eigen::Affine3d::Identity();
  //   transform_2.translation() << last_t;
  //   transform_2.rotate (last_R);

  //   // Print the transformation
  //   printf ("\nMethod #2: using an Affine3f\n");
  //   std::cout << transform_2.matrix() << std::endl;
  //   p2 = transform_2 * p1;
  //   std::cout << "p2 " << p2 << std::endl;
  //   p2 = transform_2 * p2;
  //   std::cout << "p2 " << p2 << std::endl;
  //   p2 = transform_2 * p2;
  //   std::cout << "p2 " << p2 << std::endl;

  //    printf ("\nMethod #2: using a Matrix4f\n");
  //   Eigen::Matrix4d transform_3 = Eigen::Matrix4d::Identity();
  //   transform_3.block<3,3>(0,0) = last_R;
  //   transform_3.block<3,1>(0,3) = last_t;

  //   Eigen::Vector4d p1_h, p2_h;
  //   p1_h << p1(0), p1(1), p1(2), 1.0;
  //   std::cout << transform_3 << std::endl;
  //   p2_h = transform_3 * p1_h;
  //   std::cout << "p2_h " << p2_h << std::endl;
  //   p2_h = transform_2 * p2_h;
  //   std::cout << "p2_h " << p2_h << std::endl;
  //   p2_h = transform_2 * p2_h;
  //   std::cout << "p2_h " << p2_h << std::endl;

  // }


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

#include <Eigen/Dense>
typedef double Scalar;
typedef Eigen::Matrix<Scalar, 3, 1> Vector3;

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

	int counter = 0;
	int fov_id = 0;

	void OnPose( geometry_msgs::PoseStamped const& pose ) {
		counter++;
		if (counter >= 25) {
			counter = 0;
			ROS_INFO("GOT POSE");
			fov_id += 1;
			if (fov_id >= 20) {fov_id = 0;}
			PublishFovMarker();
		}
	}

	void PublishFovMarker() {
		visualization_msgs::Marker marker;
     	marker.header.frame_id = drawing_frame;
    	marker.header.stamp = ros::Time::now();
  	   	marker.ns = "fov_side";
	    marker.id = fov_id;
  		marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
  		marker.action = visualization_msgs::Marker::ADD;
    	marker.pose.position.x = 0.0;
    	marker.pose.position.y = 0.0;
  		marker.pose.position.z = 0.0;
   		marker.pose.orientation.x = 0.0;
  		marker.pose.orientation.y = 0.0;
   		marker.pose.orientation.z = 0.0;
    	marker.pose.orientation.w = 1.0;
    	marker.pose.position.x = 0.0;
     	marker.scale.x = 1.0;
     	marker.scale.y = 1.0;
     	marker.scale.z = 1.0;
     	marker.color.r = 1.0;
     	marker.color.g = 1.0;
     	marker.color.b = 1.0;
     	marker.color.a = 1.0;

     	std::vector<Vector3> fov_corners;
     	fov_corners.push_back(Vector3(7,5.25,10)); // bottom right
     	fov_corners.push_back(Vector3(7,-5.25,10)); // top right
     	fov_corners.push_back(Vector3(-7,-5.25,10)); // top left
     	fov_corners.push_back(Vector3(-7,5.25,10)); // bottom left  

     	int j = 0;
     	for (int i = 0; i < 4; i++) {
     		j = i+1;
     		if (j == 4) {j = 0;}; // connect back around
     		BuildSideOfFOV(fov_corners.at(i), fov_corners.at(j), marker);
     	}
     	fov_pub.publish( marker );

     	marker.type = visualization_msgs::Marker::LINE_LIST;
     	marker.ns = "fov_line";
     	marker.scale.x = 0.1;
     	marker.scale.y = 0.1;
     	marker.scale.z = 0.1;
     	marker.points.clear();
     	marker.colors.clear();
     	for (int i = 0; i < 4; i++) {
     		j = i+1;
     		if (j == 4) {j = 0;}; // connect back around
     		BuildLineOfFOV(Vector3(0,0,0), fov_corners.at(i), marker);
     		BuildLineOfFOV(fov_corners.at(i), fov_corners.at(j), marker);
     	}
     	fov_pub.publish( marker );

	    
	}

	void BuildSideOfFOV(Vector3 corner_1, Vector3 corner_2, visualization_msgs::Marker& marker) {
		Vector3 corner_0 = TransformRDFtoWorld(Vector3(0,0,0));
		corner_1 = TransformRDFtoWorld(corner_1);
		corner_2 = TransformRDFtoWorld(corner_2);
		geometry_msgs::Point p;
		p.x = corner_0(0);
		p.y = corner_0(1);
  		p.z = corner_0(2);

   		geometry_msgs::Point p2 = p;
   		p2.x = corner_1(0);
   		p2.y = corner_1(1);
   		p2.z = corner_1(2);

   		geometry_msgs::Point p3 = p;
   		p3.x = corner_2(0);
   		p3.y = corner_2(1);
   		p3.z = corner_2(2);

   		marker.points.push_back(p);
   		marker.points.push_back(p2);
   		marker.points.push_back(p3);

   		std_msgs::ColorRGBA c;
   		c.r = 1.0;
   		c.g = 1.0;
   		c.b = 0.0;
   		c.a = 0.15;
   		marker.colors.push_back(c);
   		marker.colors.push_back(c);
  		marker.colors.push_back(c);       	    	
	}

	void BuildLineOfFOV(Vector3 corner_1, Vector3 corner_2, visualization_msgs::Marker& marker) {
		corner_1 = TransformRDFtoWorld(corner_1);
		corner_2 = TransformRDFtoWorld(corner_2);
		geometry_msgs::Point p;
		p.x = corner_1(0);
   		p.y = corner_1(1);
   		p.z = corner_1(2);

   		geometry_msgs::Point p2 = p;
   		p2.x = corner_2(0);
   		p2.y = corner_2(1);
   		p2.z = corner_2(2);

   		marker.points.push_back(p);
   		marker.points.push_back(p2);

   		std_msgs::ColorRGBA c;
   		c.r = 1.0;
   		c.g = 1.0;
   		c.b = 0.0;
   		c.a = 0.50;
   		marker.colors.push_back(c);
   		marker.colors.push_back(c);      	    	
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

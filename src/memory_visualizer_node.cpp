#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Point.h"

#include "tf/tf.h"
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

	void OnPose( geometry_msgs::PoseStamped const& pose ) {
		ROS_INFO("GOT POSE");
		PublishTestMarker();
	}

	void PublishTestMarker() {
		visualization_msgs::Marker marker;
     	marker.header.frame_id = drawing_frame;
    	marker.header.stamp = ros::Time::now();
  	   	marker.ns = "fov_namespace";
	    marker.id = 0;
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
    	for (int x = 0; x < 10; ++x) {
       		for (int y = 0; y < 10; ++y) {
        		for (int z = 0; z < 10; ++z) {
     				geometry_msgs::Point p;
       				p.x = x * 0.1f;
     				p.y = y * 0.1f;
	          		p.z = z * 0.1f;
	 
	           		geometry_msgs::Point p2 = p;
	           		p2.x = p.x + 0.05;
	 
	           		geometry_msgs::Point p3 = p;
	           		p3.x = p2.x;
	           		p3.z = p.z + 0.05;
	           		marker.points.push_back(p);
	           		marker.points.push_back(p2);
	           		marker.points.push_back(p3);
	 
	           		std_msgs::ColorRGBA c;
	           		c.r = x * 0.1;
	           		c.g = y * 0.1;
	           		c.b = z * 0.1;
	           		c.a = 1.0;
	           		marker.colors.push_back(c);
	           		marker.colors.push_back(c);
	          		marker.colors.push_back(c);
	        	}
	      	}
	    }
	    fov_pub.publish( marker );
	}


	std::string drawing_frame = "r200_depth_optical_frame";

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

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include <mavros_msgs/AttitudeTarget.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>

#include "tf/tf.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <mutex>
#include <cmath>
#include <time.h>
#include <stdlib.h>
#include <chrono>


class MemoryVisualizerNode {
public:

	MemoryVisualizerNode() {

		// Subscribers
		pose_sub = nh.subscribe("/pose", 1, &MemoryVisualizerNode::OnPose, this);

  	    // Publishers
		fov_pub = nh.advertise<geometry_msgs::PoseStamped>("/fov", 1);

		tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);
	}

	
private:

	void OnPose( geometry_msgs::PoseStamped const& pose ) {
		ROS_INFO("GOT POSE");
	}

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

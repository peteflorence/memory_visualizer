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
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>

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

#include "nanomap.h"

//#include <octomap_server/octomap_server.h>


class DepthHistoryLatencyEvaluatorNode {
public:

  std::ofstream ofs;

	DepthHistoryLatencyEvaluatorNode() : ofs("/home/locomotion/test_nanomap.txt", std::ofstream::out) {
		// Subscribers
		pose_sub = nh.subscribe("/pose", 100, &DepthHistoryLatencyEvaluatorNode::OnPose, this);
    smoothed_path_sub = nh.subscribe("/samros/keyposes", 100, &DepthHistoryLatencyEvaluatorNode::OnSmoothedPath, this);
    camera_info_sub = nh.subscribe("depth_camera_info", 1, &DepthHistoryLatencyEvaluatorNode::OnCameraInfo, this);
    depth_image_sub = nh.subscribe("depth_camera_pointcloud", 100, &DepthHistoryLatencyEvaluatorNode::OnDepthImage, this);

    point_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("point_cloud_after_smoothed", 100);

		tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);

    for(;;){
        try {

          tf_buffer_.lookupTransform("world", "body", 
                                        ros::Time(0), ros::Duration(30.0));
        } catch (tf2::TransformException &ex) {
          continue;
        }
        break;
    }
	}

private:

  std::vector<geometry_msgs::PoseStamped> poses_path;
  std::vector<sensor_msgs::PointCloud2ConstPtr> point_cloud_ptrs;
  NanoMap nanomap;

  bool got_camera_info = false;
  std::string depth_sensor_frame = "depth_sensor";
  void OnCameraInfo(const sensor_msgs::CameraInfo msg) {
    if (got_camera_info) {
      return;
    }
    got_camera_info = true;
    depth_sensor_frame = msg.header.frame_id;
  }

  size_t point_cloud_ctr = 0;
  void OnDepthImage(const sensor_msgs::PointCloud2ConstPtr& point_cloud_msg) {
    ROS_INFO("GOT POINT CLOUD");

    pcl::PCLPointCloud2::Ptr cloud_in (new pcl::PCLPointCloud2 ());
    pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl_conversions::toPCL(*point_cloud_msg,*cloud_in);
    pcl::fromPCLPointCloud2(*cloud_in,*xyz_cloud);

    auto t1 = std::chrono::high_resolution_clock::now();

    nanomap.AddToMergedKDTree(xyz_cloud);

    auto t2 = std::chrono::high_resolution_clock::now();
    
    nanomap.BuildNewKDTree(xyz_cloud);

    auto t3 = std::chrono::high_resolution_clock::now();


    size_t num_queries = 1000;
    auto t4 = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < num_queries; i ++) {
        //nanomap.FindNearestPointsMerged(Vector3(10,5,1));
    }
    auto t5 = std::chrono::high_resolution_clock::now();

    auto t6 = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < num_queries; i ++) {
        //nanomap.FindNearestPointsNew(Vector3(10,5,1));
    }
    auto t7 = std::chrono::high_resolution_clock::now();



    ofs << point_cloud_ctr 
           << " " << std::chrono::duration_cast<std::chrono::nanoseconds>(t2-t1).count()
           << " " << std::chrono::duration_cast<std::chrono::nanoseconds>(t3-t2).count() 
           << " " << std::chrono::duration_cast<std::chrono::nanoseconds>(t5-t4).count() 
           << " " << std::chrono::duration_cast<std::chrono::nanoseconds>(t7-t6).count() 
           << std::endl;

    point_cloud_ctr++;
    

    // size_t num_point_clouds = 90;
    // if (point_cloud_ptrs.size() < num_point_clouds) {
    //   point_cloud_ptrs.push_back(point_cloud_msg);
    //   return;
    // }

    // for (size_t i = 0; i < num_point_clouds; i++) {
    //   point_cloud_pub.publish(*point_cloud_ptrs.at(i));
    //   std::cout << "Published point cloud " << i << std::endl;
    // }
    //VoxelGrid();
    //point_cloud_ptrs.clear();
    //ros::service::call("/octomap_server/reset");

    // initialize test of building local history

    point_cloud_ctr++;
  }

  void VoxelGrid() {

    std::cout << "Converting and merging " << point_cloud_ptrs.size() << " point clouds " << std::endl;

    ros::Time time_before_merging = ros::Time::now();

    pcl::PointCloud<pcl::PointXYZ>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCLPointCloud2::Ptr cloud_in (new pcl::PCLPointCloud2 ());
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    for (size_t i = 0; i < point_cloud_ptrs.size(); i++) {  
      pcl_conversions::toPCL(*point_cloud_ptrs.at(0),*cloud_in);
      
      pcl::fromPCLPointCloud2(*cloud_in,*temp_cloud);
      *merged_cloud = *merged_cloud + *temp_cloud;
    }

    ros::Time time_after_merging = ros::Time::now();
    std::cout << "Converting and merging took " << time_after_merging - time_before_merging << std::endl;
    std::cout << "Merged point cloud has " << merged_cloud->height * merged_cloud->width << " points" << std::endl;

    ros::Time time_before_filtering = ros::Time::now();


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    // Create the filtering object
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(merged_cloud);
    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    sor.filter (*cloud_filtered);

    std::cout << "Filtered point cloud has " << cloud_filtered->height * cloud_filtered->width << " points" << std::endl;

    ros::Time time_after = ros::Time::now();

    std::cout << "took " << time_after - time_before_filtering << std::endl;

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

	std::string drawing_frame = "world";

  ros::Publisher point_cloud_pub;

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

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


class MemoryVisualizerNode {
public:

	MemoryVisualizerNode() {
		// Subscribers
		pose_sub = nh.subscribe("/pose", 100, &MemoryVisualizerNode::OnPose, this);
    smoothed_path_sub = nh.subscribe("/samros/keyposes", 100, &MemoryVisualizerNode::OnSmoothedPath, this);


  	    // Publishers
		fov_pub = nh.advertise<visualization_msgs::Marker>("fov", 0);
    poses_path_pub = nh.advertise<nav_msgs::Path>("poses_path", 0);

    point_cloud_pub_last_pose = nh.advertise<sensor_msgs::PointCloud2>("point_cloud_merged_last_pose", 0);
    point_cloud_pub_query_now = nh.advertise<sensor_msgs::PointCloud2>("point_cloud_merged_query_now", 0);
    point_cloud_pub_query_back = nh.advertise<sensor_msgs::PointCloud2>("point_cloud_merged_query_back", 0);
    point_cloud_pub_smoothed = nh.advertise<sensor_msgs::PointCloud2>("point_cloud_merged_smoothed", 0);

    camera_info_sub = nh.subscribe("depth_camera_info", 1, &MemoryVisualizerNode::OnCameraInfo, this);
    depth_image_sub = nh.subscribe("depth_camera_pointcloud", 100, &MemoryVisualizerNode::OnDepthImage, this);

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
  std::vector<Eigen::Matrix4f> transform_poses_query_now;
  std::vector<Eigen::Matrix4f> transform_poses_last_pose;

  bool initiated = false;
	int counter = 0; // this just reduces to 33 Hz from 100 Hz
  std::vector<Eigen::Matrix4d> odometries;
  //std::vector<Eigen::Matrix4d> odometries_with_noise;

  geometry_msgs::PoseStamped last_pose;
  Matrix3 BodyToRDF;
  Matrix3 BodyToRDF_inverse;
  Eigen::Matrix4d transform_body_to_world; // better if not class variable


  FovEvaluator fov_evaluator;

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
      transform_poses_last_pose.push_back(findTransform4f(last_pose)*GetRDFToBodyTransform());
      transform_poses_query_now.push_back(FindTransformNow());
      point_cloud_ptrs.push_back(point_cloud_msg);
      return;
    }
    // rotate(point_cloud_ptrs.begin(),point_cloud_ptrs.end()-1,point_cloud_ptrs.end()); // Shift vector so each move back 1
    // point_cloud_ptrs.at(0) = point_cloud_msg;

    //PublishMergedPointCloudLastPoses();
    //PublishMergedPointCloudQueryNow(); 
    PublishMergedPointCloudQueryingBack();
    PublishMergedPointCloudSmoothed();

    transform_poses_last_pose.clear();
    transform_poses_query_now.clear();
    point_cloud_ptrs.clear();

    // pcl::PointCloud<pcl::PointXYZ>::Ptr world_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Eigen::Matrix4f transform_to_world; // Your Transformation Matrix
    // transform_to_world.setIdentity();   // Set to Identity to make bottom row of Matrix 0,0,0,1
    // transform_to_world(2,3) = 50;
    // TransformToWorldFromPose(transform_to_world, point_cloud_msg, world_cloud);
    point_cloud_ctr++;
  }

  Eigen::Matrix4f FindTransformNow() {
    geometry_msgs::TransformStamped tf;
    try {
      tf = tf_buffer_.lookupTransform("world", depth_sensor_frame,
                                    ros::Time(0), ros::Duration(1/30.0));
      } catch (tf2::TransformException &ex) {
      ROS_ERROR("8 %s", ex.what());
      Eigen::Matrix4f transform;
      return transform.setIdentity();
    }
    Eigen::Quaternionf quat(tf.transform.rotation.w, tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z);
    Eigen::Matrix3f R = quat.toRotationMatrix();
    Eigen::Vector4f T = Eigen::Vector4f(tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z, 1.0); 
    Eigen::Matrix4f transform_eigen; // Your Transformation Matrix
    transform_eigen.setIdentity();   // Set to Identity to make bottom row of Matrix 0,0,0,1
    transform_eigen.block<3,3>(0,0) = R;
    transform_eigen.col(3) = T;
    return transform_eigen;
  }

 void PublishMergedPointCloudQueryNow() {
    sensor_msgs::PointCloud2 merged_cloud;
    pcl::PointCloud<pcl::PointXYZ> merged_cloud_pcl;
    sensor_msgs::PointCloud2 new_cloud;
    pcl::PointCloud<pcl::PointXYZ> new_cloud_pcl;

    geometry_msgs::TransformStamped tf;
    for (size_t i = 0; i < point_cloud_ptrs.size(); i++) {
      pcl_ros::transformPointCloud(transform_poses_query_now.at(i),*point_cloud_ptrs.at(i), new_cloud);
      pcl::fromROSMsg(new_cloud, new_cloud_pcl);
      merged_cloud_pcl = merged_cloud_pcl + new_cloud_pcl;
    }

    pcl::toROSMsg(merged_cloud_pcl, merged_cloud);
    merged_cloud.header.frame_id = "world";
    point_cloud_pub_query_now.publish(merged_cloud);
  }

  void PublishMergedPointCloudLastPoses() {
    sensor_msgs::PointCloud2 merged_cloud;
    pcl::PointCloud<pcl::PointXYZ> merged_cloud_pcl;
    sensor_msgs::PointCloud2 new_cloud;
    pcl::PointCloud<pcl::PointXYZ> new_cloud_pcl;
    geometry_msgs::TransformStamped tf;
    for (size_t i = 0; i < point_cloud_ptrs.size(); i++) {
      pcl_ros::transformPointCloud(transform_poses_last_pose.at(i),*point_cloud_ptrs.at(i), new_cloud);
      pcl::fromROSMsg(new_cloud, new_cloud_pcl);
      merged_cloud_pcl = merged_cloud_pcl + new_cloud_pcl;
    }
    pcl::toROSMsg(merged_cloud_pcl, merged_cloud);
    merged_cloud.header.frame_id = "world";
    point_cloud_pub_last_pose.publish(merged_cloud);
  }

  void PublishMergedPointCloudQueryingBack() {
    sensor_msgs::PointCloud2 merged_cloud;
    pcl::PointCloud<pcl::PointXYZ> merged_cloud_pcl;

    sensor_msgs::PointCloud2 new_cloud;
    pcl::PointCloud<pcl::PointXYZ> new_cloud_pcl;

    geometry_msgs::TransformStamped tf;
    for (size_t i = 0; i < point_cloud_ptrs.size(); i++) {
      try {
        tf = tf_buffer_.lookupTransform("world", depth_sensor_frame,
                                      point_cloud_ptrs.at(i)->header.stamp, ros::Duration(1/30.0));
        } catch (tf2::TransformException &ex) {
          ROS_ERROR("8 %s", ex.what());
        return;
      }
      Eigen::Quaternionf quat(tf.transform.rotation.w, tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z);
      Eigen::Matrix3f R = quat.toRotationMatrix();
      Eigen::Vector4f T = Eigen::Vector4f(tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z, 1.0); 
      Eigen::Matrix4f transform_eigen; // Your Transformation Matrix
      transform_eigen.setIdentity();   // Set to Identity to make bottom row of Matrix 0,0,0,1
      transform_eigen.block<3,3>(0,0) = R;
      transform_eigen.col(3) = T;
      pcl_ros::transformPointCloud(transform_eigen,*point_cloud_ptrs.at(i), new_cloud);
      pcl::fromROSMsg(new_cloud, new_cloud_pcl);
      merged_cloud_pcl = merged_cloud_pcl + new_cloud_pcl;
    }

    pcl::toROSMsg(merged_cloud_pcl, merged_cloud);
    merged_cloud.header.frame_id = "world";
    point_cloud_pub_query_back.publish(merged_cloud);
  }

  void PublishMergedPointCloudSmoothed() {
    sensor_msgs::PointCloud2 merged_cloud;
    pcl::PointCloud<pcl::PointXYZ> merged_cloud_pcl;

    sensor_msgs::PointCloud2 new_cloud;
    pcl::PointCloud<pcl::PointXYZ> new_cloud_pcl;

    geometry_msgs::TransformStamped tf;
    for (size_t i = 0; i < point_cloud_ptrs.size(); i++) {

      Eigen::Matrix4f transform_eigen;
      bool can_interpolate;
      findTransformFromSmoothedPath(point_cloud_ptrs.at(i)->header.stamp, transform_eigen, can_interpolate);
      if (can_interpolate) {
        pcl_ros::transformPointCloud(transform_eigen*GetRDFToBodyTransform(),*point_cloud_ptrs.at(i), new_cloud);
        pcl::fromROSMsg(new_cloud, new_cloud_pcl);
        merged_cloud_pcl = merged_cloud_pcl + new_cloud_pcl;
     }
    }

    pcl::toROSMsg(merged_cloud_pcl, merged_cloud);
    merged_cloud.header.frame_id = "world";
    point_cloud_pub_smoothed.publish(merged_cloud);
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
      geometry_msgs::PoseStamped pose_interpolate = InterpolateBetweenPoses(pose_before, pose_after, t_parameter, query_time);
      transform = findTransform4f(pose_interpolate);
     
    }
  }

  geometry_msgs::PoseStamped InterpolateBetweenPoses(geometry_msgs::PoseStamped const& pose_before, geometry_msgs::PoseStamped const& pose_after, double t_parameter, ros::Time time_for_pose) {
    geometry_msgs::PoseStamped interpolated_pose;
    interpolated_pose = pose_before;

    // need to actually do interpolation

    interpolated_pose.header.stamp = time_for_pose;
    return interpolated_pose;
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


  void AddToPosesPath( geometry_msgs::PoseStamped const& pose ) {

    if (  poses_path.size() < 300) {
      poses_path.push_back(pose);
    }
    else {
      rotate(poses_path.begin(),poses_path.end()-1,poses_path.end()); // Shift vector so each move back 1
      poses_path.at(0) = pose;
    }

    PublishPosesPath();
  }

  void PublishPosesPath() {
    geometry_msgs::PoseStamped pose_array [poses_path.size()];

    nav_msgs::Path path;
    path.poses.resize(poses_path.size());

    for (size_t i = 0; i < poses_path.size(); i++ ) {
      path.poses[i] = poses_path.at(i);
    }

    path.header.seq = 0;
    path.header.stamp = poses_path.at(poses_path.size()-1).header.stamp;
    path.header.frame_id = poses_path.at(poses_path.size()-1).header.frame_id;
    //std::cout << "Publishing path of length " << poses_path.size() << std::endl;
    poses_path_pub.publish(path);

  }


  size_t pose_ctr = 0;
  void OnPose( geometry_msgs::PoseStamped const& pose ) {
    //std::cout << "got pose: " << pose_ctr << std::endl;
    pose_ctr++;

    if (!initiated) {
      last_pose = pose;
      Eigen::Matrix4d transform_identity = Eigen::Matrix4d::Identity();
      transform_identity(0,3) = -1.0;
      std::cout << transform_identity << std::endl;
      for (int i = 0; i < 50; i++) {
        odometries.push_back(transform_identity);
        //odometries_with_noise.push_back(transform_identity);
      }
      initiated = true;
      return;
    }
    
    counter++;
    if (counter >= 10) {
      BodyToRDF = GetBodyToRDFRotationMatrix(); // could do only once later -- being safe for now
      BodyToRDF_inverse = BodyToRDF.inverse();

      counter = 0;
      //ROS_INFO("GOT POSE");

      AddToOdometries(findTransform(pose, last_pose));
      transform_body_to_world = findTransform(pose);
      last_pose = pose;

      PublishFovMarkers();
    }

    AddToPosesPath(pose);

  }

  nav_msgs::Path last_smoothed_path;
  void OnSmoothedPath( nav_msgs::Path const& path_msg ) {
    ROS_INFO("GOT SMOOTHED PATH");
    last_smoothed_path = path_msg;
  }

  void AddToOdometries(Eigen::Matrix4d current_transform) {
    //std::cout << "rotate and add" << std::endl;
    rotate(odometries.begin(),odometries.end()-1,odometries.end()); // Shift vector so each move back 1
    odometries.at(0) = current_transform;

    //Eigen::Matrix4d noise = Eigen::Matrix4d::Zero();
    // turning off my own noise?
    //noise(0,3) = randomNoise(0.0, 0.3);
    //noise(1,3) = randomNoise(0.0, 0.3);
    //noise(2,3) = randomNoise(0.0, 0.1);
    //rotate(odometries_with_noise.begin(),odometries_with_noise.end()-1,odometries_with_noise.end());
    //odometries_with_noise.at(0) = current_transform + noise;
  }

  double randomNoise(double mean, double sigma) {
    static std::random_device rd;
    static std::mt19937 gen(rd());

    std::normal_distribution<> d(mean, sigma);
    return d(gen);
  }


  void PublishPositionMarker(Vector3 position_world_frame, int fov_id) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = drawing_frame;
    marker.header.stamp = ros::Time::now();
    marker.ns = "fov_center";
    marker.id = fov_id;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = position_world_frame(0);
    marker.pose.position.y = position_world_frame(1);
    marker.pose.position.z = position_world_frame(2);
    marker.scale.x = 0.8 + fov_id*0.1;
    marker.scale.y = 0.8 + fov_id*0.1;
    marker.scale.z = 0.2 + fov_id*0.1/4.0;
    marker.color.a = 0.40; // Don't forget to set the alpha!

    std_msgs::ColorRGBA c = GetColorForFOV(fov_id);
    marker.color.r = c.r;
    marker.color.g = c.g;
    marker.color.b = c.b;
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

  // this function not needed
  // Eigen::Matrix4d transformFromPreviousBodyToWorldWithNoise(int fov_id) {
  //   Eigen::Matrix4d transform = transform_body_to_world;
  //   for (int i = 0; i < fov_id; i++) {
  //     transform = odometries_with_noise.at(i) * transform;
  //   }
  //   return transform;
  // }

  Eigen::Matrix4d transformFromCurrentBodyToPreviousBody(int fov_id) {
    Eigen::Matrix4d transform_world_to_previous_body_frame = invertTransform(transformFromPreviousBodyToWorld(fov_id));
    return transform_world_to_previous_body_frame * transform_body_to_world;
  }

  // this function not needed
  // Eigen::Matrix4d transformFromCurrentBodyToPreviousBodyWithNoise(int fov_id) {
  //   Eigen::Matrix4d transform_world_to_previous_body_frame = invertTransform(transformFromPreviousBodyToWorldWithNoise(fov_id));
  //   return transform_world_to_previous_body_frame * transform_body_to_world;
  // }

  Eigen::Matrix4d transformIncrementallyFromPreviousBodyToPreviousBody(int fov_id) {
    return invertTransform(odometries.at(fov_id));
  }


  void PublishFovMarkers() {
    bool publish_sampled_distributions = false;

    bool color_in_fov = false;
    for (int fov_id = 0; fov_id < 50; fov_id = fov_id + 1) {
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
      Vector3 position_current_body_frame(0.0,3.0,0.0);
      Eigen::Matrix4d transform_current_body_to_previous_body = transformFromCurrentBodyToPreviousBody(fov_id);
      Vector3 position_previous_body_frame = applyTransform(position_current_body_frame, transform_current_body_to_previous_body);
      Vector3 position_previous_rdf_frame = BodyToRDF * position_previous_body_frame;
      if (fov_evaluator.IsInFOV(position_previous_rdf_frame)) {
        color_in_fov = true;
      } 
      else {
        color_in_fov = false;
      }

      PublishFovMarker(fov_id, body, corner_1, corner_2, corner_3, corner_4, color_in_fov);
      if (publish_sampled_distributions) {
        Eigen::Matrix4d transform_to_world = transformFromPreviousBodyToWorld(fov_id);
        Vector3 position_world_frame = applyTransform(position_previous_body_frame, transform_to_world);
        PublishPositionMarker(position_world_frame, fov_id);
      }

   }


    // Checking to see if new incremental function works
      // // start                                                                        // start in world
      // Vector3 corner_1 = Vector3(7,5.25,10);
      // Vector3 corner_2 = Vector3(7,-5.25,10);
      // Vector3 corner_3 = Vector3(-7,-5.25,10);
      // Vector3 corner_4 = Vector3(-7,5.25,10);
      // Vector3 body = Vector3(0,0,0);

      // body = applyTransform(body, invertTransform(transform_body_to_world));          // transform to body
      // corner_1 = applyTransform(corner_1, invertTransform(transform_body_to_world));
      // corner_2 = applyTransform(corner_2, invertTransform(transform_body_to_world));
      // corner_3 = applyTransform(corner_3, invertTransform(transform_body_to_world));
      // corner_4 = applyTransform(corner_4, invertTransform(transform_body_to_world));

      // Eigen::Matrix4d transform = transformFromCurrentBodyToPreviousBody(30);         // transform back in pose chain to 30
      // body = applyTransform(body, transform);
      // corner_1 = applyTransform(corner_1, transform);
      // corner_2 = applyTransform(corner_2, transform);
      // corner_3 = applyTransform(corner_3, transform);
      // corner_4 = applyTransform(corner_4, transform);

      // int fov_id;
      // for (fov_id = 30; fov_id < 40; fov_id++) {
      //   Eigen::Matrix4d incremental_transform = transformIncrementallyFromPreviousBodyToPreviousBody(fov_id);  // incrementally transform to 30
      //   body = applyTransform(body, incremental_transform);
      //   corner_1 = applyTransform(corner_1, incremental_transform);
      //   corner_2 = applyTransform(corner_2, incremental_transform);
      //   corner_3 = applyTransform(corner_3, incremental_transform);
      //   corner_4 = applyTransform(corner_4, incremental_transform);
      // } 

      // transform = transformFromPreviousBodyToWorld(fov_id);                            // transform back to world
      // body = applyTransform(body, transform);
      // corner_1 = applyTransform(corner_1, transform);
      // corner_2 = applyTransform(corner_2, transform);
      // corner_3 = applyTransform(corner_3, transform);
      // corner_4 = applyTransform(corner_4, transform);

      // PublishFovMarker(fov_id, body, corner_1, corner_2, corner_3, corner_4, color_in_fov);
 }

	void PublishFovMarker(int fov_id, Vector3 body, Vector3 corner_1, Vector3 corner_2, Vector3 corner_3, Vector3 corner_4, bool color_in_fov) {
		std::vector<visualization_msgs::Marker> markers = BuildFovMarker(fov_id, body, corner_1, corner_2, corner_3, corner_4, color_in_fov);
   	fov_pub.publish( markers.at(0) );
    fov_pub.publish( markers.at(1) );
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

	ros::Publisher fov_pub;
  ros::Publisher poses_path_pub;
  ros::Publisher point_cloud_pub_last_pose;
  ros::Publisher point_cloud_pub_query_now;
  ros::Publisher point_cloud_pub_query_back;
  ros::Publisher point_cloud_pub_smoothed;

	std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
	tf2_ros::Buffer tf_buffer_;

	ros::NodeHandle nh;
};


int main(int argc, char* argv[]) {
	std::cout << "Initializing memory_visualizer_node node" << std::endl;
	ros::init(argc, argv, "MemoryVisualizerNode");
	MemoryVisualizerNode memory_visualizer_node;
  ros::Rate spin_rate(100);

  while (ros::ok()) {
    //std::cout << "spin" << std::endl;
    ros::spinOnce();
    spin_rate.sleep();
  }

}

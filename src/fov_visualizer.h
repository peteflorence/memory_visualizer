#ifndef FOV_VISUALIZER_H
#define FOV_VISUALIZER_H

#include <Eigen/Dense>
typedef double Scalar;
typedef Eigen::Matrix<Scalar, 3, 1> Vector3;
typedef Eigen::Matrix<Scalar, 4, 1> Vector4;
typedef Eigen::Matrix<Scalar, 3, 3> Matrix3;

#include <visualization_msgs/Marker.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Point.h"

void BuildSideOfFOV(Vector3 body, Vector3 corner_1, Vector3 corner_2, visualization_msgs::Marker& marker, int fov_id) {

		geometry_msgs::Point p;
		p.x = body(0);
		p.y = body(1);
  	p.z = body(2);

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
      if (fov_id == 0) {
        c.r = 1.0;
        c.g = 1.0;
        c.b = 0.0;
        c.a = 0.15;
      }
      else {
        c.r = 0.1;
        c.g = 0.1;
        c.b = 0.1;
        c.a = 0.05;  
      }
   		marker.colors.push_back(c);
   		marker.colors.push_back(c);
  		marker.colors.push_back(c);       	    	
	}

void BuildLineOfFOV(Vector3 corner_1, Vector3 corner_2, visualization_msgs::Marker& marker, int fov_id) {

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
  if (fov_id == 0) {
    c.r = 1.0;
    c.g = 1.0;
    c.b = 0.0;
    c.a = 0.50;
  }
  else {
    c.r = 0.1;
    c.g = 0.1;
    c.b = 0.1;
    c.a = 0.50;  
  }
	marker.colors.push_back(c);
	marker.colors.push_back(c);      	    	
}

#endif
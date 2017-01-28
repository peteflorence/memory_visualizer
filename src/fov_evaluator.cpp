#include "fov_evaluator.h"

bool FovEvaluator::IsInFOV(Vector3 robot_position_rdf) {
  if (IsBehind(robot_position_rdf)) {
      return false;
  }
  if (IsOutsideFOV(robot_position_rdf)) {
    return false;
  }
  if (IsBeyondSensorHorizon(robot_position_rdf)) {
    return false;
  }
  return true;
}

bool FovEvaluator::IsBehind(Vector3 robot_position_rdf) {
  return (robot_position_rdf(2) < -0.0);
}

bool FovEvaluator::IsBeyondSensorHorizon(Vector3 robot_position_rdf) {
  return (robot_position_rdf(2) > 10.0);
}

bool FovEvaluator::IsOutsideFOV(Vector3 robot_position_rdf) {
  Vector3 projected = K * robot_position_rdf;
  int pi_x = projected(0)/projected(2); 
  int pi_y = projected(1)/projected(2);

  // Checks if outside left/right FOV
  if ( (pi_x < 0) || (pi_x > (num_x_pixels - 1)) ) {
    return true;
  }
  // Checks if above top/bottom FOV
  if (pi_y < 0) {
    return true; 
  }
  if (pi_y > (num_y_pixels - 1)) {
    return true; 
  }

  // //Checks for occlusion
  // if (xyz_cloud_ptr == nullptr) {
  //   return 0.0;
  // } 
  // pcl::PointXYZ point = xyz_cloud_ptr->at(pi_x,pi_y);
  // if (std::isnan(point.z)) { 
  //    return 0.0;
  // }
  // Vector3 position_ortho_body = Vector3(point.x, point.y, point.z);
  // Vector3 position_rdf = R * position_ortho_body;
  // if( robot_position(2) >  position_rdf(2) ) {
  //   //std::cout << "OCCLUSION" << std::endl;
  //   return p_collision_occluded;
  // }

  return false;
}
#ifndef FOV_EVALUATOR_H
#define FOV_EVALUATOR_H

#include "transform_utils.h"

class FovEvaluator {
public:
  FovEvaluator() {
    //K << 304.8, 0.0, 160.06, 0.0, 304.8, 119.85, 0.0, 0.0, 1.0;
                K << 308.57684326171875, 0.0, 154.6868438720703, 0.0, 308.57684326171875, 120.21442413330078, 0.0, 0.0, 1.0;
                K/=4.0;
                K(2,2) = 1.0;
  }
  
  bool IsInFOV(Vector3 robot_position_rdf);
  bool IsBehind(Vector3 robot_position_rdf);
  bool IsBeyondSensorHorizon(Vector3 robot_position_rdf);
  bool IsOutsideFOV(Vector3 robot_position);
  
private:
  
  Matrix3 K;
  double num_x_pixels = 320/4.0;
  double num_y_pixels = 240/4.0;

};

#endif
//
// Created by francesco on 14/07/17.
//

#ifndef PROBABILISTIC_ROBOTICS_DICTPOINTS_H
#define PROBABILISTIC_ROBOTICS_DICTPOINTS_H
#include <Eigen/Dense>
#include "defs.h"

class DictPoints {
private:
  pr::Vector3fVector* landmaks = new pr::Vector3fVector();
  pr::Vector2fVector* projectedLandmarks = new pr::Vector2fVector();



public:
  void addPoint(Eigen::Vector2f p2, Eigen::Vector3f p3) {landmaks->push_back(p3);projectedLandmarks->push_back(p2);};
  pr::Vector3fVector* get3DPoints()const {return landmaks;};
  pr::Vector2fVector* get2DPoints()const {return projectedLandmarks;};
  void clear() {landmaks->clear(); projectedLandmarks->clear();};
};

#endif //PROBABILISTIC_ROBOTICS_DICTPOINTS_H

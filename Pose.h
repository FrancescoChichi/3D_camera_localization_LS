//
// Created by francesco on 14/02/17.
//

#ifndef PROBABILISTICROBOTICS_POSE_H
#define PROBABILISTICROBOTICS_POSE_H
#include <Eigen/Dense>


class Pose {
private:
  int id;
  Eigen::Isometry3f pose;
public:
  void setPose(Eigen::Isometry3f n) {pose=n;};
  void setId(int n) {id = n;};
  Eigen::Isometry3f getPose() {return pose;};
  int getId() {return id;};
};

#endif //PROBABILISTICROBOTICS_POSE_H

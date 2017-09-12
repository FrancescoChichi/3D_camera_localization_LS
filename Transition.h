//
// Created by francesco on 14/02/17.
//
#include <Eigen/Dense>

#ifndef PROBABILISTICROBOTICS_TRANSITION_H
#define PROBABILISTICROBOTICS_TRANSITION_H


class Transition {
private:
  int poseAId;
  int poseBId;
  Eigen::Isometry3f T;

public:
  void setTransition(Eigen::Isometry3f i) {T=i;};
  void setPoseAId(int n) {poseAId = n;};
  void setPoseBId(int n) {poseBId = n;};
  Eigen::Isometry3f getTransition() {return T;};
  int getPoseAId() {return poseAId;};
  int getPoseBId() {return poseBId;};

};


#endif //PROBABILISTICROBOTICS_TRANSITION_H

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
    Eigen::Matrix<float,6,6> covariance;
public:
    void setTransition(Eigen::Isometry3f i) {T=i;};
    void setPoseAId(int n) {poseAId = n;};
    void setPoseBId(int n) {poseBId = n;};
    void setCov(Eigen::Matrix<float,6,6> c) {covariance = c;};
    Eigen::Isometry3f getTransition() {return T;};
    int getPoseAId() {return poseAId;};
    int getPoseBId() {return poseBId;};
    Eigen::Matrix<float,6,6> getCov() { return covariance;};

};


#endif //PROBABILISTICROBOTICS_TRANSITION_H

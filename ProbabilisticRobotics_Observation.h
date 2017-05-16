//
// Created by francesco on 16/02/17.
//

#include <Eigen/Dense>
#include "defs.h"

#ifndef PROBABILISTICROBOTICS_OBSERVATION_H
#define PROBABILISTICROBOTICS_OBSERVATION_H


class Observation {
private:
    int poseId;
    Eigen::Isometry3f cameraPose;
    pr::Vector2fVector projectedLandmarks;
    std::vector<int> vectorId;
    std::vector<float> depth;
    Eigen::Matrix3f covariance;
    int firstId;
public:
    void setPose(Eigen::Isometry3f n){cameraPose = n;};

    void addProjectedLandmarks(Eigen::Vector2f p){
        projectedLandmarks.push_back(p);
    }
    void addId(int i) {vectorId.push_back(i);};
    void addDepth(float n) {depth.push_back(n);};
    void setPoseId(int n) {poseId = n;};
    void setCov(Eigen::Matrix3f c) {covariance = c;};
    Eigen::Isometry3f getPose(){ return cameraPose;};
    pr::Vector2fVector getProjectedLandmarks(){return projectedLandmarks;};
    std::vector<float> getDepth(){ return depth;};
    std::vector<int> getVectorId(){return vectorId;};
    int getPoseId() { return poseId;};
    Eigen::Matrix3f getCov() { return covariance;};
    void setFirstId (int i){firstId = i;};
    int getFirstId (){return firstId;};
};


#endif //PROBABILISTICROBOTICS_OBSERVATION_H
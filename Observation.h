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
    pr::Vector2fVector projectedLandmarks;
    std::vector<int> vectorId; //list of landmark's ID
    std::vector<float> depth;  //list of landmark's depth
    int firstId;
public:

    void addProjectedLandmarks(Eigen::Vector2f p){
        projectedLandmarks.push_back(p);
    }
    void addId(int i) {vectorId.push_back(i);};
    void addDepth(float n) {depth.push_back(n);};
    void setPoseId(int n) {poseId = n;};
    pr::Vector2fVector getProjectedLandmarks(){return projectedLandmarks;};
    std::vector<float> getDepth(){ return depth;};
    std::vector<int> getVectorId(){return vectorId;};
    int getPoseId() { return poseId;};
    void setFirstId (int i){firstId = i;};
    int getFirstId (){return firstId;};
};


#endif //PROBABILISTICROBOTICS_OBSERVATION_H

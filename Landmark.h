//
// Created by francesco on 14/02/17.
//

#ifndef PROBABILISTICROBOTICS_LANDMARK_H
#define PROBABILISTICROBOTICS_LANDMARK_H
#include <Eigen/Dense>


class Landmark {
    private:
        int id;
        Eigen::Vector3f p;


public:
        void setPose(Eigen::Vector3f n) {p=n;};
        void setId(int n) {id = n;};
        Eigen::Vector3f  getPose() const {return p;};
        int getId() const {return id;};
        void transform(Eigen::Isometry3f T){
            p = (T.translation().matrix() + (T.rotation().matrix()*p));

        }
};


#endif //PROBABILISTICROBOTICS_LANDMARK_H
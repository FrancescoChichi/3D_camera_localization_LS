#include <iostream>
#include <fstream>
#include <sstream>
#include "Landmark.h"
#include "Pose.h"
#include "Observation.h"
#include "Transition.h"
#include "camera.h"
#include "utils.h"
#include "points_utils.h"
#include "distance_map_correspondence_finder.h"
#include "p3p_solver.h"

using namespace std;
using namespace Eigen;
using namespace pr;

#ifndef PROBABILISTICROBOTICS_G2O_PARSER_H
#define PROBABILISTICROBOTICS_G2O_PARSER_H


class g2o_parser {

public:

    static Landmark* makeLandmark(stringstream& ss){
        Landmark* landmark;
        landmark = new Landmark();
        string token;
        Eigen::Vector3f p;

        ss>>token;
        landmark->setId(stoi(token));
        ss>>token;
        p.x() = stof(token);
        ss>>token;
        p.y() = stof(token);
        ss>>token;
        p.z() = stof(token);
        landmark->setPose(p);

        return landmark;
    }

    static Pose* makePose(stringstream& ss){
        Pose* pose;
        pose = new Pose();
        string token;
        Eigen::Quaternionf q;
        Eigen::Vector3f p;
        Eigen::Isometry3f iso = Eigen::Isometry3f::Identity();

        ss>>token;
        pose->setId(stoi(token));
        ss>>token;
        p.x() = stof(token);
        ss>>token;
        p.y() = stof(token);
        ss>>token;
        p.z() = stof(token);
        ss>>token;
        q.x() = stof(token);
        ss>>token;
        q.y() = stof(token);
        ss>>token;
        q.z() = stof(token);
        ss>>token;
        q.w() = stof(token);

        iso.translation()=p;
        iso.linear()=q.toRotationMatrix();

        pose->setPose(iso);

        return pose;
    }

    static Transition* makeTransition(stringstream& ss){
        Transition* transition;
        transition = new Transition();
        string token;
        Eigen::Matrix<float,6,6> m;
        Eigen::Quaternionf q;
        Eigen::Vector3f p;
        Eigen::Isometry3f T = Eigen::Isometry3f::Identity();

        ss>>token;
        transition->setPoseAId(stoi(token));
        ss>>token;
        transition->setPoseBId(stoi(token));
        ss>>token;
        p.x() = stof(token);
        ss>>token;
        p.y() = stof(token);
        ss>>token;
        p.z() = stof(token);
        ss>>token;
        q.x() = stof(token);
        ss>>token;
        q.y() = stof(token);
        ss>>token;
        q.z() = stof(token);
        ss>>token;
        q.w() = stof(token);

        T.translation()=p;
        T.linear()=q.toRotationMatrix();
        transition->setTransition(T);

        //cout<< "transition "<<p<<endl;
        //cout<<"transition n: "<<transition->getPoseAId()<< endl<<T.matrix()<<endl;

        for (int i=0; i<6; i++){
            for (int j=0; j<6; j++){
                if (i<=j){
                    ss>>token;
                    m(i,j)=stof(token);
                }
                else
                    m(i,j)=m(j,i);
            }
        }
        transition->setCov(m);
        return transition;
    }

    static Camera* makeCamera(stringstream& ss, Isometry3f& robotToCamera, Isometry3f& cameraToRobot, int scale, int rows, int cols){

        string token;
        Camera* camera;
        Eigen::Matrix3f c = Eigen::Matrix3f::Identity();
        Eigen::Isometry3f wtc=Eigen::Isometry3f::Identity();
        Eigen::Quaternionf q;

        float x,y,z;

        ss >> token;
        ss >> token;
        x=stof(token);
        ss >> token;
        y=stof(token);
        ss >> token;
        z=stof(token);
        ss >> token;
        q.x()=stof(token);
        ss >> token;
        q.y()=stof(token);
        ss >> token;
        q.z()=stof(token);
        ss >> token;
        q.w()=stof(token);


        Vector3f t(x,y,z);


        wtc.translation()=t;
        wtc.linear()=q.toRotationMatrix();

        robotToCamera.rotate(q);
        robotToCamera.translate(t);

        cameraToRobot = robotToCamera.inverse(Isometry);

        float fx,fy,cx,cy;

        ss >> token;
        fx=stof(token);
        ss >> token;
        fy=stof(token);
        ss >> token;
        cx=stof(token);
        ss >> token;
        cy=stof(token);


     /*   c <<
          fx, 0, cx,
            0, fy, cy,
            0, 0,   1;*/

       c<<  fx*scale, 0, cx*cols,
                0, fy*scale, cy*rows,
                0,      0,      1;

        cout  <<"camera matrix "<<endl<<c<<endl;

        camera = new Camera(rows,cols,c,wtc);
        return camera;
    }

    static void makeObservation(stringstream& ss, vector<Observation>& observations, int scale, int rows, int cols, Camera camera){

        string token;
        Eigen::Matrix3f m;
        Eigen::Vector2f p;
        int landId;
        int poseId;
        ss>>token;
        poseId = stoi(token);
        //cout<<"pose id "<<poseId<<endl;

        //se l'osservazione è nuova la aggiungo, altrimenti modifico la vecchia
        if (observations.size() > 0) {
            for (int i =( observations.size() -1 ); i >= 0; --i) {

                if (observations[i].getPoseId() == poseId) {
                    //cout << "check " << observations[i].getPoseId() << " l " << observations[i].getLandmarks().size() << endl;

                    Observation* obs = &observations[i];
                    ss >> token;
                    landId = stoi(token);
                    ss >> token; //camera id
                    ss >> token;
                    p.x() = stof(token);
                    ss >> token;
                    p.y() = stof(token);
                    ss >> token; //depth
                    obs->addDepth(stof(token));
/*
 * unproject osservazioni con camera ideale (già nel riferimento camera)
 * project punti con camera mia (senza portare i punti nel riferimento camera)
 */
                    //p.x()=((p.x()-0.5f)*scale)+(cols/2);
                    //p.y()=((p.y()-0.5f)*scale)+(rows/2);
                    //p = robotToCamera.translation().matrix() + (robotToCamera.rotation().matrix()*p);

                    Eigen::Vector3f camera_point;
                    camera_point.setZero();

                    camera.unprojectPoint(p,camera_point,stof(token));
                    camera.projectPoint(p, camera_point, true);


                    obs->addProjectedLandmarks(p);
                    obs->addId(landId);

                    for (int i = 0; i < 3; i++) {
                        for (int j = 0; j < 3; j++) {
                            if (i <= j) {
                                ss >> token;
                                m(i, j) = stof(token);
                            } else
                                m(i, j) = m(j, i);
                        }
                    }
                    obs->setCov(m);
                    return;
                }
            }
        }


        //cout<<"new obs "<<endl;
        Observation* obs = new Observation();
        obs->setPoseId(poseId);
        ss>>token;
        landId = stoi(token);
        ss>>token;
        ss >> token;
        p.x() = stof(token);
        ss>>token;
        p.y() = stof(token);
        ss>>token;
        obs->addDepth(stof(token));


        Eigen::Vector3f camera_point;
        camera_point.setZero();

        camera.unprojectPoint(p,camera_point,stof(token));
        camera.projectPoint(p, camera_point, true);

        //p.x()=((p.x()-0.5f)*scale)+(cols/2);
        //p.y()=((p.y()-0.5f)*scale)+(rows/2);
        //p = robotToCamera.translation().matrix() + (robotToCamera.rotation().matrix()*p);



        ss>>token;
        obs->addId(landId);

        obs->addProjectedLandmarks(p);
        obs->setFirstId(landId); //**************TEST**********

        //  ss>>token;
        //  obs->setDepth(stof(token));
        for (int i=0; i<3; i++){
            for (int j=0; j<3; j++){
                if (i<=j){
                    ss>>token;
                    m(i,j)=stof(token);
                }
                else
                    m(i,j)=m(j,i);
            }
        }
        obs->setCov(m);

        observations.push_back(*obs);
/*
 * unproject osservazioni con camera ideale (già nel riferimento camera)
 * project punti con camera mia (senza portare i punti nel riferimento camera)
 */
    }

};


#endif //PROBABILISTICROBOTICS_G2O_PARSER_H

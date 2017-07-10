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
#include "g2o_parser.h"


using namespace std;
using namespace Eigen;
using namespace pr;
//int rows=1080;
//int cols=1920;
int rows=480;
int cols=640;
int scale = 150;


Isometry3f robotToCamera = Eigen::Isometry3f::Identity();
Isometry3f cameraToRobot = Eigen::Isometry3f::Identity();


void poseTest(Camera *camera, const vector<Landmark>& world_points, vector<Pose>& world_poses, float density) {
    cv::namedWindow("camera_test");
    char key = 0;
    const char ESC_key = 27;
    while (key != ESC_key) {
        Vector2fVector image_points;
        // project the points on the image plane
        camera->projectPoints(image_points, world_points, false);
        RGBImage shown_image(rows, cols);
        shown_image = cv::Vec3b(255, 255, 255);
        drawPoints(shown_image, image_points, cv::Scalar(0, 0, 0), 2);


        Vector2fVector image_pose;
        camera->projectPoints(image_pose, world_poses, false);
       /* for (int j = 0; j < world_poses.size(); ++j) {
            Eigen::Vector2f p;
            p.x()=world_poses[j].getPose().translation().x();
            p.y()=world_poses[j].getPose().translation().y();
            image_pose.push_back(p);
        }*/

        drawPoints(shown_image, image_pose, cv::Scalar(255, 0, 0), 8);


        Vector3fVector lines;
        vector<Landmark> l;
        for (int i = 0; i < world_poses.size()-1; ++i) {
            putPointsOnSegment3D(lines, world_poses[i].getPose().translation(),world_poses[i+1].getPose().translation(), density); //da proiettare

        }
        for (int i = 0; i < lines.size(); ++i) {
            Landmark p;
            p.setPose(lines[i]);
            l.push_back(p);
        }
        camera->projectPoints(image_points, l, false);
        drawPoints(shown_image, image_points, cv::Scalar(255, 0, 255), 3);

        cv::imshow("camera_test", shown_image);
        Eigen::Isometry3f current_pose = camera->worldToCameraPose();

        Eigen::Isometry3f motion = Eigen::Isometry3f::Identity();

        float dr = 0.05;
        float dt = 0.1;


        key = cv::waitKey(0);

        switch (key) {
            case 'w':
                motion.translation() = Eigen::Vector3f(0, 0, -dt);
                break;
            case 's':
                motion.translation() = Eigen::Vector3f(0, 0, dt);
                break;
            case 'e':
                motion.translation() = Eigen::Vector3f(0, -dt, 0);
                break;
            case 'q':
                motion.translation() = Eigen::Vector3f(0, dt, 0);
                break;
            case 'z':
                motion.translation() = Eigen::Vector3f(-dt, 0, 0);
                break;
            case 'c':
                motion.translation() = Eigen::Vector3f(dt, 0, 0);
                break;
            case 'a':
                motion.linear() = Ry(dr);
                break;
            case 'd':
                motion.linear() = Ry(-dr);
                break;
            case 'i':
                motion.linear() = Rz(dr);
                break;
            case 'k':
                motion.linear() = Rz(-dr);
                break;
            case 'o':
                motion.linear() = Rx(dr);
                break;
            case 'u':
                motion.linear() = Rx(-dr);
                break;
            case 'h':{
                motion.setIdentity();
                cout<<"p "<<world_poses[0].getPose().matrix();
                current_pose = world_poses[0].getPose() * robotToCamera;
            }
                break;
            case ' ':{
                motion.setIdentity();
                current_pose = robotToCamera;
            }
            break;

            default:;
        }

        camera->setWorldToCameraPose(motion * current_pose);
        cout << "pose " << camera->worldToCameraPose().matrix() << endl;
       // cerr << "projction took: " << (t_end_projection - t_start_projection) << " ms" << endl;

    }
}


int main(int argc, char** argv) {
    double t_start_execution = getTime();
    string line;
    string filename;
    if(argc > 1) {
        filename = argv[2];
        std::cerr << "reading from: " << filename << std::endl;
    }
    else
     filename = "/home/francesco/g2o/bin/simulator_out.g2o";

    ifstream myfile(filename.c_str());


    //ifstream myfile("/home/francesco/Documenti/magistrale/probabilistic_robotics/project/1b-3D-Camera-Localization.g2o");//Documenti/probabilistic_robotics/project/1b-3D-Camera-Localization.g2o");
    string token;

    vector<Landmark> landmarks;
    // Vector3fVector world_points;
    vector<Observation> observations;
    vector<Pose> poses;
    vector<Transition> transitions;
    Camera *camera;

    if (myfile.is_open()) {
        while (getline(myfile, line)) {

            stringstream ssin(line);
            ssin >> token;

            if (token == "PARAMS_CAMERACALIB") {
                camera = g2o_parser::makeCamera(ssin, robotToCamera, cameraToRobot, scale, rows, cols);
            } else if (token == "VERTEX_TRACKXYZ") {
                landmarks.push_back(*g2o_parser::makeLandmark(ssin));
            } else if (token == "VERTEX_SE3:QUAT") {
                poses.push_back(*g2o_parser::makePose(ssin));
            } else if (token == "EDGE_SE3:QUAT") {
                transitions.push_back(*g2o_parser::makeTransition(ssin));
            } else if (token == "EDGE_PROJECT_DEPTH") {
                g2o_parser::makeObservation(ssin, observations, scale, rows, cols, *camera);
            }
        }
        myfile.close();
    } else cout << "Unable to open file";

    /*******************************************TEST**************************************************/

    ///cameraTest(camera, landmarks);

    ///correspondenceFinderTest(camera, landmarks);

    /*************************************************************************************************/




/*


    //Eigen::Vector2f ip = observations[0].getProjectedLandmarks()[0];
    Eigen::Vector2f ip;

    ip.setZero();
    Eigen::Vector3f wp;
    for (int i = 0; i < landmarks.size(); ++i) {
        if (landmarks[i].getId()==57){
            wp = landmarks[i].getPose();
            cout<<"landmark "<<i<<"   " << wp;

        }
    }


    camera->projectPoint(ip,wp);

    cout<<"punto proiettato "<<ip<<endl;




    Eigen::Vector3f p = (camera->worldToCameraPose()*wp);
    camera->unprojectPoint(ip,wp, p.z() );


    Eigen::Isometry3f current_camera_pose = camera->worldToCameraPose();
  //  Eigen::Isometry3f motion = transitions[0].getTransition();

    //camera->setWorldToCameraPose((current_camera_pose * motion));

    Isometry3f cameraToWorld = camera->worldToCameraPose().inverse(Isometry);
    //camera->unprojectPoint(ip,wp,observations[0].getDepth()[0]);

    wp = cameraToWorld * wp;
    cout<<"unprojcterd "<<wp<<endl;


   // cout<<"landmark "<<landmarks[9].getPose();

*/

if(true)
    for (int i = 0; i<transitions.size(); ++i) {//transitions.size()
        //applicare transizione A->B alla camera
      //std::cout<<camera->worldToCameraPose().inverse(Isometry).matrix()<<std::endl;

        cout << "transition from " << transitions[i].getPoseAId() << " to " << transitions[i].getPoseBId() << endl;

        Eigen::Isometry3f current_camera_pose = camera->worldToCameraPose();
        Eigen::Isometry3f motion = transitions[i].getTransition();


        camera->setWorldToCameraPose((current_camera_pose * cameraToRobot * motion * robotToCamera));


        Isometry3f cameraToWorld = (camera->worldToCameraPose()).inverse(Isometry);

        //cout<<"cam "<<endl<<camera->worldToCameraPose().matrix()<<endl;

        Vector2fVector image_points;

        Observation Z_to;

        for (int j = 0; j < observations.size(); ++j) {  //prendo l'osservazione relativa alla mia posa
            if (observations[j].getPoseId() == transitions[i].getPoseBId()) {
                Z_to = observations[j];
                break;
            }
        }

        Vector2fVector observed_points = Z_to.getProjectedLandmarks();
        float maxDepth = 0;
        float range = 0.2;
        for (int j = 0; j < Z_to.getProjectedLandmarks().size(); ++j) { //calcolo la depth massima
            if (maxDepth < Z_to.getDepth()[j])
                maxDepth = Z_to.getDepth()[j];
        }

        maxDepth = maxDepth + range;

        vector<Landmark> world_points;
        for (int j = 0; j < landmarks.size(); ++j) { //filtro i landmarks in base alla depth
            Eigen::Vector3f p = cameraToWorld * landmarks[j].getPose();//camera->worldToCameraPose()* landmarks[j].getPose(); ***************
            p = camera->cameraMatrix() * p;
            if (p.z() <= maxDepth)
                world_points.push_back(landmarks[j]);
        }


        // project the points on the image plane

        camera->projectPoints(image_points, landmarks, false);


        //cameraTestProjected(camera, observed_points);

        // camera->projectPoints(observed_points, obs, false);

        cout << endl << endl << "**************************************************************************" << endl;
        cout << "landmarks projected " << image_points.size() << endl;
        // cout<<"observations in the image " << image_points[1]<<endl;
        cout << "observations projected " << observed_points.size() << endl;

        //correspondenceFinderTest(camera,obs);
        //correspondenceFinderTest(camera,landmarks);
        //cameraTest(camera,obs);
        //cameraTest(camera,landmarks);

        //data association
        // construct a correspondence finder
        DistanceMapCorrespondenceFinder correspondence_finder;
        float max_distance = 20;

        correspondence_finder.init(image_points,
                                   rows,
                                   cols,
                                   max_distance);

        correspondence_finder.compute(observed_points);

        //show corrispondence
        if(true){
            RGBImage img1(rows, cols);
            img1 = cv::Vec3b(255, 255, 255);


            drawPoints(img1, image_points, cv::Scalar(255, 0, 0), 3);


            drawPoints(img1, observed_points, cv::Scalar(255, 0, 255), 3);


            RGBImage shown_image;
            drawDistanceMap(shown_image,
                            correspondence_finder.distanceImage(),
                            correspondence_finder.maxDistance() - 1);

            drawPoints(shown_image, image_points, cv::Scalar(0, 0, 255), 3);
            drawPoints(shown_image, observed_points, cv::Scalar(0, 255, 0), 3);
            drawCorrespondences(shown_image,
                                image_points,
                                observed_points,
                                correspondence_finder.correspondences());
          cv::imshow("observation", img1);

          cv::imshow("distance map ", shown_image);
          cv::waitKey(0);

        }

        //calcolo errore tra punti proiettati - punti calcolati con transazione
        //least square

        // construct a solver
        P3PSolver solver;
        solver.setKernelThreshold(1000);



        for (int k = 0; k < 1; ++k) {

            //dcerr<<"error: "<<solver.numInliers()<<endl;
            //cout<<"l "<<world_points.size()<<endl;
            solver.init(*camera, image_points , observed_points);
            solver.oneRound(correspondence_finder.correspondences(), false);

           // camera->setWorldToCameraPose(solver.camera().worldToCameraPose());
            *camera = solver.camera();




          camera->projectPoints(image_points, landmarks, false);


          correspondence_finder.init(image_points,
                                     rows,
                                     cols,
                                     max_distance);

          correspondence_finder.compute(observed_points);
        }




        cout<<"associations "<<correspondence_finder.correspondences().size()<<endl;
       // cout<<"camera pose:"<<endl<<camera->worldToCameraPose().matrix()<<endl;
        cout<<endl<<endl<<"***************************************************************************"<<endl;


        poses[i].setPose(camera->worldToCameraPose());





    }


    double t_end_execution = getTime();
    cerr << "execution took: " << (t_end_execution - t_start_execution) << " ms" << endl;


    Isometry3f sky_view = Eigen::Isometry3f::Identity();
    Matrix3f R;


    Vector3f t;
    t<<-6.49545, -7.22359,7.92861;
    //t<<-100, 30,22.9583;

    R<<-0.0494831, -0.99571, 0.0781914,
            -0.998766, 0.0496694, 0.000437762,
            -0.0043196, -0.0780733, -0.996939;

    sky_view.translation()=t;
    sky_view.linear()=R;

   /* Eigen::Matrix3f c = Eigen::Matrix3f::Identity();
    c<<  scale, 0, cols/2,
            0, scale, rows/2,
            0,      0,      1;*/

    Camera *sky_view_camera = new Camera(rows,cols, camera->cameraMatrix(), sky_view);

    poseTest(sky_view_camera, landmarks, poses, 10);
    //poseTest(camera, landmarks, poses, 10);


}

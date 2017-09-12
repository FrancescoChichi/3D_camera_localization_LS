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
#include "DictPoints.h"

using namespace std;
using namespace Eigen;
using namespace pr;

/** parameters **/

int rows=480; //1080
int cols=640; //1920
int scale = 150;
float rangeDepth = 0.3; //depth of the range filter
float max_distance = 20; //dimension of distance map circle
int lsIteration = 10; //least square iteration
int kernelThreshold = 1000; //least square kernel threshold
bool showCorrispondence = true;
bool showLSStep = false;
bool ls = false;

Isometry3f robotToCamera = Eigen::Isometry3f::Identity();
Isometry3f cameraToRobot = Eigen::Isometry3f::Identity();


void poseTest(Camera *camera, const vector<Landmark>& world_points, vector<Pose>& world_poses, float density) {
  cv::namedWindow("camera_test");
  char key = 0;
  const char ESC_key = 27;
  while (key != ESC_key) {

    DictPoints points;
    camera->projectPoints(points, world_points, false);
    RGBImage shown_image(rows, cols);
    shown_image = cv::Vec3b(255, 255, 255);
    drawPoints(shown_image, *points.get2DPoints(), cv::Scalar(0, 0, 0), 2);


    Vector2fVector image_pose;
    camera->projectPoints(image_pose, world_poses, false);

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
    camera->projectPoints(points, l, false);
    drawPoints(shown_image, *points.get2DPoints(), cv::Scalar(255, 0, 255), 3);

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

  string token;

  vector<Landmark> landmarks;
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


  for (int i = 0; i<transitions.size(); ++i) {//transitions.size()
    //applicare transizione A->B alla camera
    cout << "transition from " << transitions[i].getPoseAId() << " to " << transitions[i].getPoseBId() << endl;

    Eigen::Isometry3f current_camera_pose = camera->worldToCameraPose();
    Eigen::Isometry3f motion = transitions[i].getTransition();

    cout << "transition " << motion.matrix() << endl;

    camera->setWorldToCameraPose(current_camera_pose * cameraToRobot * motion * robotToCamera);

    Isometry3f cameraToWorld = (camera->worldToCameraPose()).inverse(Isometry);

    DictPoints points;

    Observation Z_to;

    for (int j = 0; j < observations.size(); ++j) {  //prendo le osservazioni relative alla mia posa
      if (observations[j].getPoseId() == transitions[i].getPoseBId()) {
        Z_to = observations[j];
        break;
      }
    }

    Vector2fVector observed_points = Z_to.getProjectedLandmarks();
    float maxDepth = 0;


    for (int j = 0; j < Z_to.getProjectedLandmarks().size(); ++j) { //calcolo la depth massima
      if (maxDepth < Z_to.getDepth()[j])
        maxDepth = Z_to.getDepth()[j];
    }

    maxDepth = maxDepth + rangeDepth;

    vector<Landmark> world_points;
    Vector3fVector landmarks_points;
    for (int j = 0; j < landmarks.size(); ++j) { //filtro i landmarks in base alla depth
      Eigen::Vector3f p =
          cameraToWorld * landmarks[j].getPose();//camera->worldToCameraPose()* landmarks[j].getPose(); ***************
      p = camera->cameraMatrix() * p;
      if (p.z() <= maxDepth) {
        world_points.push_back(landmarks[j]);
        landmarks_points.push_back(landmarks[j].getPose());
      }
    }

    camera->projectPoints(points, world_points, false);

    //data association
    // construct a correspondence finder
    DistanceMapCorrespondenceFinder correspondence_finder;

    correspondence_finder.init(observed_points,
                               rows,
                               cols,
                               max_distance);

    correspondence_finder.compute(*points.get2DPoints());

    //show corrispondence
    if (showCorrispondence) {
      RGBImage img1(rows, cols);
      img1 = cv::Vec3b(255, 255, 255);


      drawPoints(img1, *points.get2DPoints(), cv::Scalar(255, 0, 0), 3);


      drawPoints(img1, observed_points, cv::Scalar(255, 0, 255), 3);


      RGBImage shown_image;
      drawDistanceMap(shown_image,
                      correspondence_finder.distanceImage(),
                      correspondence_finder.maxDistance() - 1);

      drawPoints(shown_image, *points.get2DPoints(), cv::Scalar(0, 0, 255), 3);
      drawPoints(shown_image, observed_points, cv::Scalar(0, 255, 0), 3);
      drawCorrespondences(shown_image,
                          observed_points,
                          *points.get2DPoints(),
                          correspondence_finder.correspondences());
      cv::imshow("observation", img1);
      cv::imshow("distance map ", shown_image);
      cv::waitKey(0);

    }

    // construct a solver
    P3PSolver solver;
    solver.setKernelThreshold(kernelThreshold);

    for (int k = 0; k < ls * lsIteration; ++k) {

      //************one LS iteration ************
      solver.init(*camera, points, observed_points);
      solver.oneRound(correspondence_finder.correspondences(), false);
      *camera = solver.camera();
      //*****************************************

      camera->projectPoints(points, world_points, false);
      correspondence_finder.init(observed_points,
                                 rows,
                                 cols,
                                 max_distance);
      correspondence_finder.compute(*points.get2DPoints()); //landmarks projected


      if (showLSStep) {
        RGBImage img1(rows, cols);
        img1 = cv::Vec3b(255, 255, 255);


        drawPoints(img1, *points.get2DPoints(), cv::Scalar(255, 0, 0), 3);


        drawPoints(img1, observed_points, cv::Scalar(255, 0, 255), 3);


        RGBImage shown_image;
        drawDistanceMap(shown_image,
                        correspondence_finder.distanceImage(),
                        correspondence_finder.maxDistance() - 1);

        drawPoints(shown_image, *points.get2DPoints(), cv::Scalar(0, 0, 255), 3);
        drawPoints(shown_image, observed_points, cv::Scalar(0, 255, 0), 3);
        drawCorrespondences(shown_image,
                            observed_points,
                            *points.get2DPoints(),
                            correspondence_finder.correspondences());
        cv::imshow("observation", img1);

        cv::imshow("distance map ", shown_image);
        cv::waitKey(0);
      }

    }

    cout << "associations " << correspondence_finder.correspondences().size() << endl;
    // cout<<"camera pose:"<<endl<<camera->worldToCameraPose().matrix()<<endl;
    cout << endl << endl << "***************************************************************************" << endl;

    poses[i].setPose(camera->worldToCameraPose());


    double t_end_execution = getTime();
    cerr << "execution took: " << (t_end_execution - t_start_execution) << " ms" << endl;

    Isometry3f sky_view = Eigen::Isometry3f::Identity();
    Matrix3f R;
    Vector3f t;
    t << -6.49545, -7.22359, 7.92861;

    R << -0.0494831, -0.99571, 0.0781914,
        -0.998766, 0.0496694, 0.000437762,
        -0.0043196, -0.0780733, -0.996939;

    sky_view.translation() = t;
    sky_view.linear() = R;

    Camera *sky_view_camera = new Camera(rows, cols, camera->cameraMatrix(), sky_view);


    poseTest(sky_view_camera, landmarks, poses, 10);
  }
}

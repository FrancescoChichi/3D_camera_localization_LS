
#ifndef PROBABILISTIC_ROBOTICS_TEST_H
#define PROBABILISTIC_ROBOTICS_TEST_H
#include "Landmark.h"
#include "camera.h"
#include "points_utils.h"

using namespace std;
using namespace Eigen;
using namespace pr;

void poseTest(Camera *camera, Isometry3f robotToCamera, const vector<Landmark>& world_points, vector<Pose>& world_poses, float density) {
  cv::namedWindow("camera_test");
  char key = 0;
  const char ESC_key = 27;
  while (key != ESC_key) {

    DictPoints points;
    camera->projectPoints(points, world_points, false);
    RGBImage shown_image(camera->getRows(), camera->getCols());
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


#endif //PROBABILISTIC_ROBOTICS_TEST_H
#include "camera.h"
#include "DictPoints.h"

namespace pr {
  Camera::Camera(int rows,
                 int cols,
                 const Eigen::Matrix3f& camera_matrix,
                 const Eigen::Isometry3f& world_to_camera_pose):
    _rows(rows),
    _cols(cols),
    _camera_matrix(camera_matrix),
    _camera_matrix_inv(cameraMatrix().inverse()),
    _world_to_camera_pose(world_to_camera_pose),
    _camera_pose_to_world(world_to_camera_pose.inverse()){}

  int Camera::projectPoints(DictPoints points,
                            const std::vector<Landmark>& world_points,
                            bool keep_indices){

    points.get2DPoints()->resize(world_points.size());
    points.get3DPoints()->resize(world_points.size());

    DictPoints projectedPoints;

    int num_image_points=0;
    const Eigen::Vector2f point_outside(-1,-1);
    int num_points_inside=0;

    for(size_t i=0; i<world_points.size(); i++){
      const Eigen::Vector3f world_point=world_points[i].getPose();
      Eigen::Vector2f& image_point= (*points.get2DPoints())[num_image_points];

      bool is_inside=projectPoint(image_point,world_point, false);

      if (is_inside) {
        projectedPoints.addPoint(image_point, world_point);
        num_points_inside++;
      }
      else
        image_point=point_outside;
      if (keep_indices||is_inside){
        num_image_points++;
      }
    }

    *points.get2DPoints()=*projectedPoints.get2DPoints();
    *points.get3DPoints()=*projectedPoints.get3DPoints();

    return num_points_inside;
  }

  int Camera::projectPoints(Vector2fVector& image_points,
                            std::vector<Pose>& world_pose,
                            bool keep_indices){
      image_points.resize(world_pose.size());
      int num_image_points=0;
      const Eigen::Vector2f point_outside(-1,-1);
      int num_points_inside=0;
      for(size_t i=0; i<world_pose.size(); i++){
          Eigen::Vector3f world_point=world_pose[i].getPose().translation();
          Eigen::Vector2f& image_point=image_points[num_image_points];
          bool is_inside=projectPoint(image_point,world_point, false);
          if (is_inside)
              num_points_inside++;
          else
              image_point=point_outside;
          if (keep_indices||is_inside){
              num_image_points++;
          }
      }
      image_points.resize(num_image_points);
      return num_points_inside;
  }
}

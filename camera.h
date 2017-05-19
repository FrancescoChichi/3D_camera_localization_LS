#pragma once
#include "defs.h"
#include "Landmark.h"
#include "Pose.h"

namespace pr {
    /**
       simple pinhole camera class.
       Has
       - the position (world with respect to camers)
       - the camera matrix
       - the size of the image plane in pixels
       Supports simple projection operations
     */
    class Camera{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        //~ ctor, initialized a camera according to the arguments
        Camera(int rows=100,
               int cols=100,
               const Eigen::Matrix3f& camera_matrix=Eigen::Matrix3f::Identity(),
               const Eigen::Isometry3f& world_to_camera_pose=Eigen::Isometry3f::Identity());

        //! projects a single point on the image plane
        //! @returns false if the point is behind the camera or outside the plane
        inline  bool projectPoint(Eigen::Vector2f& image_point,
                                  const Eigen::Vector3f& world_point,
                                    bool observation){
            image_point.setZero();
            Eigen::Vector3f camera_point;
            if(!observation)
                camera_point=_world_to_camera_pose.inverse(Eigen::Isometry)*world_point;
            else
                camera_point = world_point;
            //std::cout<<"cam point "<<camera_point<<std::endl;

          //std::cout<<"a " <<std::endl;
            if (camera_point.z()<=0)
                return false;
            //std::cout<<"cam matrix "<<_camera_matrix.matrix()<<std::endl;
            Eigen::Vector3f projected_point=_camera_matrix*camera_point;
            image_point=projected_point.head<2>()*(1./projected_point.z());
         // std::cout<<"b"<<image_point<<std::endl;
         // std::cerr<<"cols "<<_cols<<" rows "<<_rows<<std::endl;
          if(image_point.x()<0 || image_point.x()>_cols-1)
                return false;
         // std::cout<<"c"<<std::endl;

          if(image_point.y()<0 || image_point.y()>_rows-1)
                return false;
            return true;
        }

        inline  bool projectPose(Eigen::Vector2f& image_point,
                                  const Eigen::Vector3f& world_point){
            Eigen::Vector3f camera_point=_world_to_camera_pose*world_point;
            if (camera_point.z()<=0)
                return false;
            Eigen::Vector3f projected_point=_camera_matrix*camera_point;
            image_point=projected_point.head<2>()*(1./projected_point.z());
            if(image_point.x()<0 || image_point.x()>_cols-1)
                return false;
            if(image_point.y()<0 || image_point.y()>_rows-1)
                return false;
            return true;
        }



        inline  bool projectLandmark(Eigen::Vector2f& image_point,
                                  const Eigen::Vector3f& world_point, float threshold){
            Eigen::Vector3f camera_point=_world_to_camera_pose*world_point;
            if (camera_point.z()<=0)
                return false;
            Eigen::Vector3f projected_point=_camera_matrix*camera_point;
            if (projected_point.z()>threshold)
                return false;
            image_point=projected_point.head<2>()*(1./projected_point.z());
            if(image_point.x()<0 || image_point.x()>_cols-1)
                return false;
            if(image_point.y()<0 || image_point.y()>_rows-1)
                return false;
            return true;
        }


        inline void unprojectPoint(const Eigen::Vector2f& image_point,
                                   Eigen::Vector3f& camera_point,
                                   float depth){


            camera_point.x()=(image_point.x()-.5f)*depth;
            camera_point.y()=(image_point.y()-.5f)*depth;
            camera_point.z()=depth;


          /*  camera_point.x()=(image_point.x()-_camera_matrix.col(2)[0])*depth/_camera_matrix.col(0)[0];
            camera_point.y()=(image_point.y()-_camera_matrix.col(2)[1])*depth/_camera_matrix.col(1)[1];
            camera_point.z()=depth;*/

        }

        //! projects a bunch of world points on the image
        //! @param image_points: the points on the image
        //! @param world points: the input world points
        //! @param keep_indices: if true, image_points has the same size of world points
        //! Invalid points are marked with (-1, -1). If false only the points in the set are returned
        //! @returns the number of points that fall inside the image
        int projectPoints(Vector2fVector& image_points,
                          const std::vector<Landmark>& world_points,
                          bool keep_indices=false);

        int projectPoses(Vector2fVector& image_points,
                                 std::vector<Pose>& world_pose,
                                 bool keep_indices);
        int projectPoints(Vector2fVector& image_points,
                          std::vector<Pose>& world_pose,
                          bool keep_indices);

        int projectLandmarks(Vector2fVector& image_points,
                         const std::vector<Landmark>& world_points,
                         bool keep_indices,
                         float threshold);



        inline const Eigen::Isometry3f& worldToCameraPose() const {return _world_to_camera_pose;}
        inline void setWorldToCameraPose(const Eigen::Isometry3f& pose)  {_world_to_camera_pose=pose;}
        inline const Eigen::Matrix3f& cameraMatrix() const {return _camera_matrix;}



    protected:
        int _rows; // image_size
        int _cols; //
        Eigen::Matrix3f _camera_matrix;
        Eigen::Matrix3f _camera_matrix_inv;
        Eigen::Isometry3f _world_to_camera_pose;
        Eigen::Isometry3f _camera_pose_to_world;

    };
}

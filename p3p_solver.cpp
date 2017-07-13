#include <Eigen/Cholesky>
#include <iostream>
#include "p3p_solver.h"
#include "ProjectPoint.h"

namespace pr {
  
  P3PSolver::P3PSolver(){
    _predicted=0;
    _reference=0;
    _damping=1;
    _min_num_inliers=0;
    _num_inliers=0;
    _kernel_thereshold=1000; // 33 pixels
  }

  void P3PSolver::init(const Camera& camera_,
	    const Vector2fVector& predicted,
	    const Vector2fVector& reference){
    _camera=camera_;
    _predicted=&predicted;
    _reference=&reference;
  }


  void P3PSolver::init(const Camera& camera_,
                       const std::vector<Landmark>& world_points,
                       const Vector2fVector& reference_image_points){
    _camera=camera_;
    _world_points=&world_points;
    _reference_image_points=&reference_image_points;
  }

  bool P3PSolver::errorAndJacobian(Eigen::Vector2f& error,
                                   Matrix2_6f& jacobian,
                                   const Eigen::Vector2f& predicted,
                                   const Eigen::Vector2f& reference){

    //std::cerr<<"predicted: "<<predicted<< " ref: " << reference<<std::endl;
    error=predicted-reference;
    //std::cerr<<"error "<<error<<std::endl;

    return true;
  }

  bool P3PSolver::errorAndJacobian(Eigen::Vector2f& error,
                                   Matrix2_6f& jacobian,
                                   const Eigen::Vector3f& world_point,
                                   const Eigen::Vector2f& reference_image_point){
    // compute the prediction
    Eigen::Vector2f predicted_image_point;
    bool is_good=_camera.projectPoint(predicted_image_point, world_point, false);
    if (! is_good)
      return false;
    error=predicted_image_point-reference_image_point;

    // compute the jacobian of the transformation
    Eigen::Vector3f camera_point=_camera.worldToCameraPose()*world_point;
    Matrix3_6f Jr=Eigen::Matrix<float, 3,6>::Zero();
    Jr.block<3,3>(0,0).setIdentity();
    Jr.block<3,3>(0,3)=skew(-camera_point);

    Eigen::Vector3f phom=_camera.cameraMatrix()*camera_point;
    float iz=1./phom.z();
    float iz2=iz*iz;
    // jacobian of the projection
    Matrix2_3f Jp;
    Jp <<
       iz, 0, -phom.x()*iz2,
        0, iz, -phom.y()*iz2;

    jacobian=Jp*_camera.cameraMatrix()*Jr;
    return true;
  }
  void P3PSolver::linearize(const IntPairVector& correspondences, bool keep_outliers){
    _H.setZero();
    _b.setZero();
    _num_inliers=0;
    _chi_inliers=0;
    _chi_outliers=0;

    for (const IntPair& correspondence: correspondences){
      Eigen::Vector2f e;
      Matrix2_6f J;
      int ref_idx=correspondence.first;
      //std::cerr<<"ref_idx "<<ref_idx<<std::endl;
      //std::cerr<<"len "<<(*_reference).size()<<std::endl;

      int curr_idx=correspondence.second;
      bool inside=errorAndJacobian(e,
                                   J,
                                   ((*_predicted)[curr_idx]),
                                   (*_reference)[ref_idx]);

      //  std::cout<<"chieedewdede "<<std::endl;

      if (! inside)
        continue;

       // std::cout<<"point "<<((*_world_points)[curr_idx]).getId()<<std::endl;
      float chi=e.dot(e);
      float lambda=1;
      bool is_inlier=true;
        //std::cout<<"chi "<<chi<<std::endl;
      if (chi>_kernel_thereshold){
        lambda=AD::sqrt(_kernel_thereshold/chi);
        is_inlier=false;
        _chi_outliers+=chi;
      }
      else {
        _chi_inliers+=chi;
        _num_inliers++;
      }
      //std::cerr<<"chi "<<chi<<std::endl;

      //std::cerr<<"inliers "<<_num_inliers<<std::endl;
      if (is_inlier || keep_outliers){
        _H+=J.transpose()*J*lambda;
        _b+=J.transpose()*e*lambda;
      }
    }
  }

  bool P3PSolver::oneRound(const IntPairVector& correspondences, bool keep_outliers){
    using namespace std;
     // std::cout<<"inliers prima "<<_num_inliers<<std::endl;

    linearize(correspondences, keep_outliers);
    //std::cout<<"inliers dopo "<<_num_inliers<<std::endl;

    _H+=Matrix6f::Identity()*_damping;

    if(_num_inliers<_min_num_inliers) {
      cerr << "too few inliers, skipping" << endl;
      return false;
    }
    //compute a solution
    Vector6f dx = _H.ldlt().solve(-_b);
    _camera.setWorldToCameraPose(v2tEuler(dx)*_camera.worldToCameraPose());
    return true;
  }
}

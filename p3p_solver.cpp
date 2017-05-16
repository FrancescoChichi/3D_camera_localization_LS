#include <Eigen/Cholesky>
#include <iostream>
#include "p3p_solver.h"
#include "ProjectPoint.h"

namespace pr {
  
  P3PSolver::P3PSolver(){
    _world_points=0;
    _reference_image_points=0;
    _damping=1;
    _min_num_inliers=0;
    _num_inliers=0;
    _kernel_thereshold=1000; // 33 pixels
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
				 const Eigen::Vector3f& world_point,
				 const Eigen::Vector2f& reference_image_point){
      // compute the prediction
      Eigen::Vector2f predicted_image_point;
      bool is_good=_camera.projectPoint(predicted_image_point, world_point, false);
      if (! is_good)
	    return false;
      error=predicted_image_point-reference_image_point;

      // compute the jacobian of the transformation
      Eigen::Vector3f camera_point=_camera.worldToCameraPose().inverse(Eigen::Isometry)*world_point;//**************************************.inverse(Eigen::Isometry)
      Matrix3_6f Jr=Eigen::Matrix<float, 3,6>::Zero();
      Jr.block<3,3>(0,0).setIdentity();
      Jr.block<3,3>(0,3)=skew(-camera_point);





      Eigen::Isometry3f robot = _camera.worldToCameraPose();
      Eigen::Quaternionf q(robot.rotation());

      std::cerr<<"mat: "<<robot.matrix()<<std::endl;
      std::cerr<<"point: "<<camera_point<<std::endl;
      std::cerr<<"quat: x: "<<q.x()<<" y: " << q.y()<< " z: " <<q.z()<<std::endl;
      std::cerr<<"pose: x: "<<robot.translation().x()<<" y: " << robot.translation().y()<< " z: " <<robot.translation().z()<<std::endl;


    ADMultivariateFunction<double, ProjectPoint> ad_project_point;

    ad_project_point.point<<camera_point.x(),
                            camera_point.y(),
                            camera_point.z();
    // ad_project_point.point<<1,2,3;

    Eigen::Matrix<double, 6, 1> v;
    v << robot.translation().x(), robot.translation().y(), robot.translation().z(),
         q.x(), q.y(), q.z();

    Eigen::Matrix<double, 3, 1> output;
    Eigen::Matrix<double, 3, 6> J;

    ad_project_point(&output[0], &v[0]);

    J=ad_project_point.jacobian(&v[0]);

    std::cerr << "output: " << std::endl;
    std::cerr << output.transpose() << std::endl;
    std::cerr << "jacobian: " << std::endl;
    std::cerr << jacobian << std::endl;











    /* std::cerr<<"error: "<<error<<std::endl;
     std::cerr<<"point: "<<camera_point<<std::endl;
     std::cerr<<"Jr: "<<Jr.matrix()<<std::endl;*/
      Eigen::Vector3f phom=_camera.cameraMatrix()*camera_point;
      float iz=1./phom.z();
      float iz2=iz*iz;
      // jacobian of the projection
      Matrix2_3f Jp;
      Jp << 
	    iz, 0, -phom.x()*iz2,
	    0, iz, -phom.y()*iz2;
      //std::cerr<<"Jp: "<<Jp.matrix()<<std::endl;

      jacobian=Jp*_camera.cameraMatrix()*Jr;

      std::cerr<<"Jacobian2: "<<jacobian.matrix()<<std::endl;

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
      int curr_idx=correspondence.second;
      bool inside=errorAndJacobian(e,
				   J,
                   ((*_world_points)[curr_idx]).getPose(),
                   (*_reference_image_points)[ref_idx]);
      if (! inside)
	    continue;

       // std::cout<<"point "<<((*_world_points)[curr_idx]).getId()<<std::endl;
      float chi=e.dot(e);
      float lambda=1;
      bool is_inlier=true;
      if (chi>_kernel_thereshold){
        lambda=AD::sqrt(_kernel_thereshold/chi);
        is_inlier=false;
        _chi_outliers+=chi;
          }
      else {
        _chi_inliers+=chi;
        _num_inliers++;
      }
        //  std::cout<<"chi "<<chi<<std::endl;

         // std::cout<<"inliers "<<_num_inliers<<std::endl;
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

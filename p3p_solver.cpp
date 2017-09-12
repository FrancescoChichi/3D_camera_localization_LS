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
    _kernel_threshold=1000; // 33 pixels
  }

  void P3PSolver::init(const Camera& camera_,
	    const DictPoints& predicted,
	    const Vector2fVector& reference){
    _camera=camera_;
    _predicted=&predicted;
    _reference=&reference;
  }

  void P3PSolver::errorAndJacobian(Eigen::Vector2f& error,
                                   Matrix2_6f& jacobian,
                                   const Eigen::Vector3f& world_point,
                                   const Eigen::Vector2f& predicted_image_point,
                                   const Eigen::Vector2f& reference_image_point){

    error=predicted_image_point-reference_image_point;

    // compute the numeric jacobian of the transformation
    jacobian.setZero();
    for (int i = 0; i < 6; ++i) {
      jacobian.block<2,1>(0,i)<<derivative(i, _epsilon, _camera.worldToCameraPose().inverse(Eigen::Isometry), _camera.cameraMatrix(),
                                           (Vector3f &) world_point, reference_image_point);
    }
    std::cerr<<"jacobian "<<std::endl<<jacobian<<std::endl;
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

      errorAndJacobian(e,
                       J,
                       ((*_predicted).get3DPoints()->at(curr_idx)),
                       ((*_predicted).get2DPoints()->at(curr_idx)), //(*points.get2DPoints())
                       (*_reference)[ref_idx]);

      float chi=e.dot(e);
      float lambda=1;
      bool is_inlier=true;

      if (chi>_kernel_threshold){
        lambda=AD::sqrt(_kernel_threshold/chi);
        is_inlier=false;
        _chi_outliers+=chi;
      }
      else {
        _chi_inliers+=chi;
        _num_inliers++;
      }
      if (is_inlier || keep_outliers){
        _H+=J.transpose()*J*lambda;
        _b+=J.transpose()*e*lambda;
      }
    }
  }

  bool P3PSolver::oneRound(const IntPairVector& correspondences, bool keep_outliers){
    using namespace std;

    linearize(correspondences, keep_outliers);
    _H+=Matrix6f::Identity()*_damping;

    if(_num_inliers<_min_num_inliers) {
      cerr << "too few inliers, skipping" << endl;
      return false;
    }
    //compute a solution
    Vector6f dx = _H.ldlt().solve(-_b);
    _camera.setWorldToCameraPose(v2tEuler(dx).inverse(Eigen::Isometry)*_camera.worldToCameraPose());
    return true;
  }
}

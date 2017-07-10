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
  

  bool P3PSolver::errorAndJacobian(Eigen::Vector2f& error,
				 Matrix2_6f& jacobian,
				 const Eigen::Vector2f& predicted,
				 const Eigen::Vector2f& reference){


      error=predicted-reference;

      

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
                   ((*_predicted)[curr_idx]),
                   (*_reference)[ref_idx]);

      //  std::cout<<"chieedewdede "<<std::endl;

        if (! inside)
	    continue;

       // std::cout<<"point "<<((*_world_points)[curr_idx]).getId()<<std::endl;
      float chi=e.dot(e);
      float lambda=1;
      bool is_inlier=true;
        std::cout<<"chi "<<chi<<std::endl;
      if (chi>_kernel_thereshold){
        lambda=AD::sqrt(_kernel_thereshold/chi);
        is_inlier=false;
        _chi_outliers+=chi;
          }
      else {
        _chi_inliers+=chi;
        _num_inliers++;
      }
          std::cerr<<"chi "<<chi<<std::endl;

          std::cerr<<"inliers "<<_num_inliers<<std::endl;
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

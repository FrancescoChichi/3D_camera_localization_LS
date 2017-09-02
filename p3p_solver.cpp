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
	    const DictPoints& predicted,
	    const Vector2fVector& reference){
    _camera=camera_;
    _predicted=&predicted;
    _reference=&reference;
  }


  void P3PSolver::init(const Camera& camera_,
                       const Vector3fVector& world_points,
                       const Vector2fVector& reference_image_points){
    _camera=camera_;
    _world_points=&world_points;
    _reference=&reference_image_points;
  }

  bool P3PSolver::errorAndJacobian(Eigen::Vector2f& error,
                                   Matrix2_6f& jacobian,
                                   const Eigen::Vector2f& predicted,
                                   const Eigen::Vector2f& reference){

    //std::cerr<<"predicted: "<<predicted<< " ref: " << reference<<std::endl;
    error=predicted-reference;
    std::cerr<<"error "<<error<<std::endl;

    std::cerr<<"predicted "<<std::endl<<predicted<<std::endl<< " reference "<<std::endl<<reference<<std::endl;

    //std::cerr<<"error "<<error<<std::endl;

    return true;
  }

  void P3PSolver::errorAndJacobian(Eigen::Vector2f& error,
                                   Matrix2_6f& jacobian,
                                   const Eigen::Vector3f& world_point,
                                   const Eigen::Vector2f& predicted_image_point,
                                   const Eigen::Vector2f& reference_image_point){
    // compute the prediction
    //Eigen::Vector2f predicted_image_point;
    //bool is_good=_camera.projectPoint(predicted_image_point, world_point, false);

    //if(is_good)std::cerr<<"good "<<is_good<<endl;
    //if (! is_good)
      //return false;

    error=predicted_image_point-reference_image_point;
    std::cerr<<"error "<<error<<std::endl;

    std::cerr<<"predicted "<<std::endl<<predicted_image_point<<std::endl<< " reference "<<std::endl<<reference_image_point<<std::endl;


    // compute the jacobian of the transformation

    //jacobian.matrix()<<0,1,2,3,4,5,6,7,8,9,10,11;
    //std::cerr<<jacobian.matrix()<<std::endl;
    //std::cerr<<jacobian.block<2,1>(0,5)<<std::endl;

    jacobian.setZero();
    for (int i = 0; i < 6; ++i) {
      jacobian.block<2,1>(0,i)<<derivative(i, _epsilon, _camera.worldToCameraPose().inverse(Eigen::Isometry), _camera.cameraMatrix(),
                                           (Vector3f &) world_point, reference_image_point);
    }
    std::cerr<<"jacobian "<<std::endl<<jacobian<<std::endl;

    //return true;
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

      std::cerr<<"len _wp "<<_reference->size()<<std::endl;
      std::cerr<<"curr_idx  "<<curr_idx<<std::endl;
      std::cerr<<"ref_idx  "<<ref_idx<<std::endl;

      errorAndJacobian(e,
                                J,
                                ((*_predicted).get3DPoints()->at(curr_idx)),
                                ((*_predicted).get2DPoints()->at(curr_idx)), //(*points.get2DPoints())
                                (*_reference)[ref_idx]);
/*                                   const Eigen::Vector3f& world_point,
                                 const Eigen::Vector2f& predicted_image_point,
                                 const Eigen::Vector2f& reference_image_point){*/

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

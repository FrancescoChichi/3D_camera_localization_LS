#include "ad_geometry.h"
#include <iostream>
#include <Eigen/StdVector>
#include "ad_multivariate.h"

using namespace std;
using namespace AD;
using namespace Eigen;

typedef AD::Vector2ad<float> Vector2adf;
typedef AD::Vector3ad<float> Vector3adf;
typedef AD::Vector6ad<float> Vector6adf;
typedef AD::Matrix3ad<float> Matrix3adf;
typedef AD::DualValue_<float> DualValuef;
typedef AD::Isometry3ad<float> Isometry3adf;



template <typename Scalar_>
class ProjectPoint: public MultivariateFunction<Scalar_, 6, 3>{

public:
  void operator()(Scalar_* output, const Scalar_* input){

    // this maps the memory area in the input array to an
    // Eigen vector of dimension 6
    // if you feel uncomfy, you can
    // allocate an array
    //   Vector6<Scalar_> robot_pose;
    // fill the elements with a for loop
    //    for (int i=0; i<6; i++) robot_pose[i]=input[i];
    Eigen::Map<const Eigen::Matrix<Scalar_,6,1> > robot_pose(input);

    // this maps the memory area in the output array to an
    // Eigen vector of dimension 3
    // Changing the Eigen object would result
    // in doing side effect to the memory area
    Eigen::Map<Eigen::Matrix<Scalar_,3,1> > projected_point(output);


    // compute the rotation matrix and translation vector
    // encoded in robot_pose
    Isometry3<Scalar_> robot_pose_matrix=v2t<Scalar_>(robot_pose);

    // compute the position of the point w.r.t. the world,
    // by multiplying it by a transformation matrix
    projected_point=robot_pose_matrix*point;
    //std::cout<<robot_pose_matrix.matrix()<<std::endl;
  }

  // this is the parameter
  Eigen::Matrix<Scalar_, 3, 1> point;

};
#ifndef ESTIMATE_KINECT_TRANSFORM_POINT
#define ESTIMATE_KINECT_TRANSFORM_POINT

#include <cminpack.h>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry> 
#include <Eigen/StdVector>

typedef std::vector<Eigen::Vector3f,Eigen::aligned_allocator<Eigen::Vector3f> > PointVector;

class EstimateKinectTransform
{
  public:
    void addData(Eigen::Transform<float, 3, Eigen::Affine> baselink, Eigen::Transform<float, 3, Eigen::Affine> target);
    Eigen::Transform<float, 3, Eigen::Affine> getTransform();
    void computeTransform();
    
    // Function for cminpack
    static int functionToOptimize (void *p, int m, int n, const double *x, double *fvec, int iflag);
    
    // Computer error between 2 poses
    double computeError(int m, Eigen::Transform<float, 3, Eigen::Affine> t_base_kinect);
    
    double computeTotalCost();
  
  private:
    double reprojectionError(Eigen::Vector3f point_obs, Eigen::Vector3f point_calc);
  
  
    std::vector<Eigen::Transform<float, 3, Eigen::Affine>,Eigen::aligned_allocator<Eigen::Transform<float, 3, Eigen::Affine> > > baselink_data, target_data;
    std::vector<Eigen::Transform<float, 3, Eigen::Affine>,Eigen::aligned_allocator<Eigen::Transform<float, 3, Eigen::Affine> > > baselink_diff_data, target_diff_data;
    Eigen::Transform<float, 3, Eigen::Affine> result_transform;
    
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif

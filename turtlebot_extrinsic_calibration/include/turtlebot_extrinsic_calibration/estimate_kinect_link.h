#ifndef _ESTIMATE_KINECT_LINK_
#define _ESTIMATE_KINECT_LINK_

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry> 
#include <Eigen/StdVector>
#include <cminpack.h>

typedef std::vector<Eigen::Vector3f,Eigen::aligned_allocator<Eigen::Vector3f> > PointVector;

const double pi = std::acos(-1.0);

class ObjectPose
{
  public:
    ObjectPose() : translation(0,0,0), orientation(1,0,0,0) { }
    ObjectPose(Vector3f trans, Quaternionf orient): translation(trans), orientation(orient) { }
    ObjectPose(Quaternionf orient) : translation(0,0,0),  orientation(orient) { }
    ObjectPose(Vector3f trans) : translation(trans), orientation(1,0,0,0) { }
    
    Eigen::Vector3f translation;
    Eigen::Quaternionf orientation;
    
    Eigen::Transform<float,3,Eigen::Affine> transform()
    {
      Eigen::Transform<float,3,Eigen::Affine> newtrans(orientation);
      newtrans.pretranslate(translation);
      
      return newtrans;
    }
};

class EstimateKinectTransform
{
  public:
    EstimateKinectTransform() : result_transform(Eigen::Translation3f(0,0,0)) {}
  
    void addData(ObjectPose base_pose_, PointVector target_points_);
    
    void computeTransform();
    double computeTotalCost();
    Eigen::Transform<float, 3, Eigen::Affine> getTransform();
    
    static int functionToOptimize (void *p, int m, int n, const double *x, double *fvec, int iflag);
    double computeError(int m, Eigen::Transform<float, 3, Eigen::Affine> t_base_kinect);
  
  public:
  //private:
    double reprojectionError(Eigen::Vector3f point_obs, Eigen::Vector3f point_calc);
  
    std::vector<ObjectPose,Eigen::aligned_allocator<ObjectPose> > base_pose;
    std::vector<PointVector> target_points;
    Eigen::Transform<float, 3, Eigen::Affine> result_transform;
    
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif

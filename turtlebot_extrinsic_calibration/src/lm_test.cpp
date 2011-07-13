#include <cminpack.h>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry> 
#include <Eigen/StdVector>

using namespace Eigen;

class EstimateKinectTransform
{
  public:
    void addData(Transform<float, 3, Affine> baselink, Transform<float, 3, Affine> target);
    Transform<float, 3, Affine> getTransform();
    void computeTransform();
    
    // Function for cminpack
    static int functionToOptimize (void *p, int m, int n, const double *x, double *fvec, int iflag);
    
    // Computer error between 2 poses
    double computeError(int m, Transform<float, 3, Affine> t_base_kinect);
  
  private:
    std::vector<Transform<float, 3, Affine>,Eigen::aligned_allocator<Transform<float, 3, Affine> > > baselink_data, target_data;
    std::vector<Transform<float, 3, Affine>,Eigen::aligned_allocator<Transform<float, 3, Affine> > > baselink_diff_data, target_diff_data;
    Transform<float, 3, Affine> result_transform;
    
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

double EstimateKinectTransform::computeError(int m, Transform<float, 3, Affine> t_base_kinect)
{
  Transform<float, 3, Affine> t_base_1_2, t_kinect_obj_1,
        t_kinect_1_2, t_kinect_obj_2_calc, t_kinect_obj_2_obs, t_world_base, t_base_diff;
        
  // Make everything a transform.
  // TODO: Check all of these for sizes.
  t_base_diff = baselink_diff_data[m];
  t_kinect_obj_1 = target_data[m];
  t_kinect_obj_2_obs = target_data[m+1];
  //t_

  // First, figure out what the target_diff should be.
  t_kinect_1_2 = t_base_kinect * t_base_diff * t_base_kinect.inverse();
  t_kinect_obj_2_calc = t_kinect_obj_1 * t_kinect_1_2.inverse();
  
  
  // Then, figure out distance between actual target_diff and calculated.
  return 0;
}

void EstimateKinectTransform::addData(Transform<float, 3, Affine> baselink, Transform<float, 3, Affine> target)
{
  // Compute difference between this pose and last pose, and add that to the data.
  if (baselink_data.size() > 0)
  {
    // Add a new diff point.
    baselink_diff_data.push_back(baselink*baselink_data.back().inverse());
    target_diff_data.push_back(target*target_data.back().inverse());
  }
  
  // Add poses to the vector (this can soon be just last pose, kept as vector for debugging).
  baselink_data.push_back(baselink);
  target_data.push_back(target);
}



/** \brief Estimate a rigid rotation transformation between a source and a target point cloud using an iterative
  * non-linear Levenberg-Marquardt approach.
  * \param cloud_src the source point cloud dataset
  * \param cloud_tgt the target point cloud dataset
  * \param transformation_matrix the resultant transformation matrix
  */
void EstimateKinectTransform::computeTransform()
{
  int n_unknowns = 6;      // 6 unknowns: 3 translation + 3 rotation (quaternion)

  // Check that data is the same size.

  // Need at least 4 data points (is that true?)

  int m = baselink_diff_data.size(); // Number of points ***
  double *fvec = new double[m];     // Output array of length m (error array)
  int *iwa = new int[n_unknowns];   // Integer work array of length n
  int lwa = m * n_unknowns + 5 * n_unknowns + m;  // Some other work array
  double *wa = new double[lwa];     // Work array of length iwa

  // Set the initial solution
  double *x = new double[n_unknowns];
  // Translation estimates - initial guess
  x[0] = 0; x[1] = 0; x[2] = 0;
  // Rotation estimates - initial guess quaternion: x-y-z-w
  x[3] = 0; x[4] = 0; x[5] = 0;

  // Set tol to the square root of the machine. Unless high solutions are required, these are the recommended settings.
  double tol = sqrt (dpmpar (1));

  // Optimize using forward-difference approximation LM
  //int info = lmdif1 (&pcl::IterativeClosestPointNonLinear<PointSource, PointTarget>::functionToOptimize, this, m, n_unknowns, x, fvec, tol, iwa, wa, lwa);
  int info = lmdif1 (&EstimateKinectTransform::functionToOptimize, this, m, n_unknowns, x, fvec, tol, iwa, wa, lwa); // Function call

  // Compute the norm of the residuals
  /*ROS_DEBUG ("[pcl::%s::estimateRigidTransformationLM] LM solver finished with exit code %i, having a residual norm of %g. ",
             //"\nFinal solution: [%f %f %f %f] [%f %f %f]", 
             getClassName ().c_str (), info, enorm (m, fvec));
             //x[0], x[1], x[2], x[3], x[4], x[5], x[6]); */

  delete [] wa; delete [] fvec;
  
  result_transform.matrix().setZero ();

  // Return the correct transformation
  // Compute w from the unit quaternion
  Eigen::Quaternionf q (0, x[3], x[4], x[5]);
  q.w () = sqrt (1 - q.dot (q));
  result_transform.rotation() = q.toRotationMatrix ();

  Eigen::Vector3f t (x[4], x[5], x[6]);
  result_transform.translation() = t;

  delete[] iwa;
  delete[] x;
}

/** \brief Cost function to be minimized
  * \param p a pointer to our data structure array
  * \param m the number of functions
  * \param n the number of variables
  * \param x a pointer to the variables array
  * \param fvec a pointer to the resultant functions evaluations
  * \param iflag set to -1 inside the function to terminate execution
  */
int EstimateKinectTransform::functionToOptimize (void *p, int m, int n, const double *x, double *fvec, int iflag)
{
  // Redo this section with Transform objects. Double check the proper way to rotate/pre-rotate. 
  EstimateKinectTransform *model = (EstimateKinectTransform*)p;


  // Copy the rotation and translation components
  Eigen::Vector3f t (x[0], x[1], x[2]);
  // Compute w from the unit quaternion
  Eigen::Quaternionf q (0, x[3], x[4], x[5]);
  q.w () = sqrt (1 - q.dot (q));
  // If the quaternion is used to rotate several points (>1) then it is much more efficient to first convert it
  // to a 3x3 Matrix. Comparison of the operation cost for n transformations:
  // * Quaternion: 30n
  // * Via a Matrix3: 24 + 15n
  Transform<float, 3, Affine> transformation_matrix;
  transformation_matrix.matrix().setZero();
  transformation_matrix.rotation() = q.toRotationMatrix ();
  transformation_matrix.translation() = t;

  // Go through each data point and calculate error. ***
  // How do you calculate error between 2 poses?
  for (int i = 0; i < m; ++i)
  {
    fvec[i] = model->computeError(i, transformation_matrix); // Cost function
  }
  return (0);
}

int main( )
{
  EstimateKinectTransform est;
  
}

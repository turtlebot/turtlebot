#include <turtlebot_extrinsic_calibration/estimate_kinect_transform.h>

using namespace Eigen;
using namespace std;

double EstimateKinectTransform::computeError(int m, Transform<float, 3, Affine> t_base_kinect)
{
  Transform<float, 3, Affine> t_base_1_2, t_kinect_obj_1,
        t_kinect_1_2, t_kinect_obj_2_calc, t_kinect_obj_2_obs, t_world_base, t_base_diff;
        
  // Make everything a transform.
  // TODO: Check all of these for sizes.
  t_base_diff = baselink_diff_data[m];
  t_kinect_obj_1 = target_data[m];
  t_kinect_obj_2_obs = target_data[m+1];
  
  /*t_base_diff = baselink_data[m];
  t_kinect_obj_1 = target_data[0];
  t_kinect_obj_2_obs = target_data[m];*/

  // First, figure out what the target_diff should be.
  //t_kinect_1_2 = t_base_kinect * t_base_diff * t_base_kinect.inverse();
  //t_kinect_obj_2_calc = t_kinect_obj_1 * t_kinect_1_2.inverse();
  
  // Kinect(old) to kinect(new): (kinect(old)->kinect(new)): kinect->base, base(old)->base(new), base->kinect
  t_kinect_1_2 = t_base_kinect * t_base_diff * t_base_kinect.inverse();
  // View of target (currently) (kinect(new)->target(new)): kinect(new)->kinect(old), kinect(old)->target(old) 
  t_kinect_obj_2_calc = t_kinect_obj_1*t_kinect_1_2.inverse();
  
  
  // Then, figure out distance between actual target_diff and calculated.
  /* cout << "Observed position: " << endl << t_kinect_obj_2_obs.matrix() << endl
      << "Estimated position: " << endl << t_kinect_obj_2_calc.matrix() << endl; */
  
  // Project 3 points in each plane: (0, 0, 0), (0, 1, 0), (0, 0, 1) 
  // Are these the points I want to project? Does it matter?
  // Do this in a less stupid manner.
  Vector3f point0(0,0,0), point1(0,1,0), point2(0,0,1);
  Vector3f point0_obs, point1_obs, point2_obs;
  Vector3f point0_calc, point1_calc, point2_calc;
  
  point0_obs = t_kinect_obj_2_obs.inverse()*point0;
  point1_obs = t_kinect_obj_2_obs.inverse()*point1;
  point2_obs = t_kinect_obj_2_obs.inverse()*point2;
  
  point0_calc = t_kinect_obj_2_calc.inverse()*point0;
  point1_calc = t_kinect_obj_2_calc.inverse()*point1;
  point2_calc = t_kinect_obj_2_calc.inverse()*point2;
  
  double error = reprojectionError(point0_obs, point0_calc) + 
                 reprojectionError(point1_obs, point1_calc) +
                 reprojectionError(point2_obs, point2_calc);
  
  // cout << "Error: " << error << endl;
  
  Quaternionf quat_obs(t_kinect_obj_2_obs.rotation());
  Quaternionf quat_calc(t_kinect_obj_2_calc.rotation());
  quat_obs.norm();
  quat_calc.norm();
  
  
  
  double quaternion_distance = sqrt(pow(quat_obs.x()-quat_obs.x(),2)+pow(quat_obs.y()-quat_obs.y(),2)+pow(quat_obs.z()-quat_obs.z(),2));
  double linear_distance = (t_kinect_obj_2_obs.translation() - t_kinect_obj_2_calc.translation()).norm();
  double stupid_error = quaternion_distance + 10*linear_distance;
  
  return error;
}

double EstimateKinectTransform::reprojectionError(Vector3f point_obs, Vector3f point_calc)
{
  // Squared distance between 2 vectors is the squared norm of their difference.
  // Do we want norm or squaredNorm? I think norm...
  return (point_obs - point_calc).norm();
}

double EstimateKinectTransform::computeTotalCost()
{
  int m = baselink_diff_data.size();
  double sum = 0;
  for (int i = 0; i < m; ++i)
  {
     sum += computeError(i, result_transform); // Cost function
  }

  return sum;
}

void EstimateKinectTransform::addData(Transform<float, 3, Affine> baselink, Transform<float, 3, Affine> target)
{
  // Compute difference between this pose and last pose, and add that to the data.
  if (baselink_data.size() > 0)
  {
    // Add a new diff point.
    cout << "Baselink_diff: " << (baselink*baselink_data.back().inverse()).matrix() << endl;
    baselink_diff_data.push_back(baselink*baselink_data.back().inverse());
    target_diff_data.push_back(target*target_data.back().inverse());
  }
  
  // Add poses to the vector (this can soon be just last pose, kept as vector for debugging).
  baselink_data.push_back(baselink);
  target_data.push_back(target);
  
  cout << "Added data!" << endl;
}

Eigen::Transform<float, 3, Eigen::Affine> EstimateKinectTransform::getTransform()
{
  return result_transform;
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


  double precost = computeTotalCost();
  cout << "About to run LM. Total pre-cost: " << precost/m << endl;
  // Optimize using forward-difference approximation LM
  //int info = lmdif1 (&pcl::IterativeClosestPointNonLinear<PointSource, PointTarget>::functionToOptimize, this, m, n_unknowns, x, fvec, tol, iwa, wa, lwa);
  int info = lmdif1 (&EstimateKinectTransform::functionToOptimize, this, m, n_unknowns, x, fvec, tol, iwa, wa, lwa); // Function call

  // Compute the norm of the residuals
  /*ROS_DEBUG ("[pcl::%s::estimateRigidTransformationLM] LM solver finished with exit code %i, having a residual norm of %g. ",
             //"\nFinal solution: [%f %f %f %f] [%f %f %f]", 
             getClassName ().c_str (), info, enorm (m, fvec));
             //x[0], x[1], x[2], x[3], x[4], x[5], x[6]); */

  cout << "Residuals: " << enorm(m,fvec) << endl;

  delete [] wa; delete [] fvec;
  
  result_transform = Translation<float, 3>(0,0,0);
  
  // Return the correct transformation
  // Compute w from the unit quaternion
  Eigen::Quaternionf q (0, x[3], x[4], x[5]);
  q.w () = sqrt (1 - q.dot (q));
  result_transform.matrix().topLeftCorner<3, 3>() = q.toRotationMatrix ();

  Eigen::Vector3f t (x[0], x[1], x[2]);
  result_transform.translation() = t;
  
  delete[] iwa;
  delete[] x;
  
  double postcost = computeTotalCost();
  
  cout << "Post running LM. Status: " << info << " Pre-cost: " << precost/m << " Post-cost: " << postcost/m << endl;
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
  // cout << "Q: " << q.toRotationMatrix () << endl;
  
  Transform<float, 3, Affine> transformation_matrix;
  transformation_matrix = Translation<float, 3>(0,0,0);
  transformation_matrix.matrix().topLeftCorner<3, 3>() = q.toRotationMatrix();
  transformation_matrix.translation() = t;
  
  // cout << "Transformation matrix: " << endl << transformation_matrix.matrix() << endl;

  // Go through each data point and calculate error. ***
  // How do you calculate error between 2 poses?
  for (int i = 0; i < m; ++i)
  {
    fvec[i] = model->computeError(i, transformation_matrix); // Cost function
  }
  return (0);
}

/*int main( )
{
  EstimateKinectTransform est;
  
}*/

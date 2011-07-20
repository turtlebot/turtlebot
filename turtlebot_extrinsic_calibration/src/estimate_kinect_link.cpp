#include <turtlebot_extrinsic_calibration/estimate_kinect_link.h>

using namespace Eigen;
using namespace std;

double EstimateKinectTransform::computeError(int m, Transform<float, 3, Affine> base_kinect)
{
  if (base_pose.size() < (unsigned int)m+2)
    return 0;
  
  // Get the transforms and points for this iteration.
  Transform<float,3,Affine> kinect_transform = base_pose[m+1].transform()*base_kinect;
  PointVector kinect_points = target_points[m+1];
  
  double error = 0;
  
  for (unsigned int j=0; j < base_pose.size(); j++)
  {
    if (j == m+1)
      continue;
    Transform<float,3,Affine> previous_transform = base_pose[j].transform()*base_kinect;
    PointVector previous_points = target_points[j];
  
  
    for (unsigned int i=0; i < kinect_points.size(); i++)
    { 
      Vector3f previous_point_world = previous_transform*previous_points[i];
      //cout << "Previous world point: " << previous_point_world.transpose() << endl;
      
      Vector3f current_point_world = kinect_transform*kinect_points[i];
      //cout << "Current world point: " << current_point_world.transpose() << endl;
      
      error += reprojectionError(previous_point_world, current_point_world);
    }
  }
  
  return error;
}

double EstimateKinectTransform::reprojectionError(Vector3f point_obs, Vector3f point_calc)
{
  // Squared distance between 2 vectors is the squared norm of their difference.
  return (point_obs - point_calc).norm();
}

double EstimateKinectTransform::computeTotalCost()
{
  int m = base_pose.size();
  double sum = 0;
  for (int i = 0; i < m; ++i)
  {
     sum += computeError(i, result_transform); // Cost function
  }

  return sum;
}

double EstimateKinectTransform::computeTotalCost(Eigen::Transform<float, 3, Eigen::Affine> transform)
{
  int m = base_pose.size();
  double sum = 0;
  for (int i = 0; i < m; ++i)
  {
     sum += computeError(i, transform); // Cost function
  }

  return sum;
}

void EstimateKinectTransform::addData(ObjectPose base_pose_, PointVector target_points_)
{
  base_pose.push_back(base_pose_);
  //cout << "Base pose added: " << endl << base_pose_.transform().matrix() << endl;
  target_points.push_back(target_points_);
  //for (int i=0; i < target_points.back().size(); i++)
  //{
  //  cout << "Point " << i << ": " << target_points.back()[i].transpose() << endl;
  //}
}

Eigen::Transform<float, 3, Eigen::Affine> EstimateKinectTransform::getTransform()
{
  return result_transform;
}

void EstimateKinectTransform::computeTransform(Eigen::Transform<float, 3, Eigen::Affine> guess)
{
  int n_unknowns = 6;      // 6 unknowns: 3 translation + 3 rotation (quaternion)

  // Check that data is the same size.

  // Need at least 4 data points (is that true?)
  //if (base_pose.size()-1 < 4) return;

  int m = base_pose.size()-1; // Number of points ***
  double *fvec = new double[m];     // Output array of length m (error array)
  int *iwa = new int[n_unknowns];   // Integer work array of length n
  int lwa = m * n_unknowns + 5 * n_unknowns + m;  // Some other work array
  double *wa = new double[lwa];     // Work array of length iwa

  // Set the initial solution
  double *x = new double[n_unknowns];
  // Translation estimates - initial guess
  x[0] = guess.translation().x(); x[1] = guess.translation().y(); x[2] = guess.translation().z();
  
  Quaternionf guess_quat(guess.rotation());
  // Rotation estimates - initial guess quaternion: x-y-z-w
  x[3] = guess_quat.x(); x[4] = guess_quat.y(); x[5] = guess_quat.z();

  // Set tol to the square root of the machine. Unless high solutions are required, these are the recommended settings.
  double tol = sqrt (dpmpar (1));

  double precost = computeTotalCost();
  //cout << "About to run LM. Total pre-cost: " << precost/m << endl;
  // Optimize using forward-difference approximation LM
  //int info = lmdif1 (&EstimateKinectTransform::functionToOptimize, this, m, n_unknowns, x, fvec, tol, iwa, wa, lwa); // Function call


  double ftol = sqrt (dpmpar (1));
  double xtol = sqrt (dpmpar (1));
  double gtol = sqrt (dpmpar (1));
  int maxfev = 1000;
  double epsfcn = 1e-5; // This is probs the variable to change.
  double *diag = new double[n_unknowns];
  int mode = 1;
  double factor = 100;
  int nprint = -1;
  double *wa1 = new double[n_unknowns];
  double *wa2 = new double[n_unknowns];
  double *wa3 = new double[n_unknowns];
  double *wa4 = new double[m];
  int nfev = 0;
  double *fjac = new double[m*n_unknowns];
  int ldfjac = m; // WTF IS THIS???
  int *ipvt = new int[n_unknowns];
  double *qtf = new double[n_unknowns];
  
  int info = lmdif (&EstimateKinectTransform::functionToOptimize, this, m, n_unknowns, x, fvec, 
        ftol, xtol, gtol, maxfev, epsfcn, diag, mode, factor, nprint, &nfev, fjac, ldfjac, ipvt,
	      qtf, wa1, wa2, wa3, wa4 );


  //cout << "Residuals: " << enorm(m,fvec) << endl;

  delete [] wa; delete [] fvec;
  delete [] wa1; delete [] wa2; delete [] wa3; delete [] wa4; delete [] fjac;
  delete [] diag; delete [] ipvt; delete [] qtf;
  
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
  
  //cout << "Post running LM. Status: " << info << " Pre-cost: " << precost/m << " Post-cost: " << postcost/m << endl;
}

int EstimateKinectTransform::functionToOptimize (void *p, int m, int n, const double *x, double *fvec, int iflag)
{
  EstimateKinectTransform *model = (EstimateKinectTransform*)p;

  // Copy the rotation and translation components
  Eigen::Vector3f t (x[0], x[1], x[2]);
  // Compute w from the unit quaternion
  Eigen::Quaternionf q (0, x[3], x[4], x[5]);
  q.w () = sqrt (1 - q.dot (q));
  
  Transform<float, 3, Affine> transformation_matrix;
  transformation_matrix = Translation<float, 3>(0,0,0);
  transformation_matrix.matrix().topLeftCorner<3, 3>() = q.toRotationMatrix();
  transformation_matrix.translation() = t;
  
  //cout << "Transformation matrix: " << endl << transformation_matrix.matrix() << endl;

  double totalcost = 0;
  for (int i = 0; i < m; ++i)
  {
    fvec[i] = model->computeError(i, transformation_matrix); // Cost function
    totalcost += fvec[i];
  }
  
  //cout << "Total cost at step: " << totalcost << endl;
  return (0);
}


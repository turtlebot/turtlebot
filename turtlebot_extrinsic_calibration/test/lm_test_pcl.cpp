#include <cminpack.h>

/** \brief Estimate a rigid rotation transformation between a source and a target point cloud using an iterative
  * non-linear Levenberg-Marquardt approach.
  * \param cloud_src the source point cloud dataset
  * \param cloud_tgt the target point cloud dataset
  * \param transformation_matrix the resultant transformation matrix
  */
template <typename PointSource, typename PointTarget> void
pcl::IterativeClosestPointNonLinear<PointSource, PointTarget>::estimateRigidTransformationLM (
      const PointCloudSource &cloud_src, const PointCloudTarget &cloud_tgt, Eigen::Matrix4f &transformation_matrix)
{
  boost::mutex::scoped_lock lock (tmp_mutex_);

  int n_unknowns = 6;      // 6 unknowns: 3 translation + 3 rotation (quaternion)

  if (cloud_src.points.size () != cloud_tgt.points.size ())
  {
    ROS_ERROR ("[pcl::IterativeClosestPointNonLinear::estimateRigidTransformationLM] Number or points in source (%zu) differs than target (%zu)!", cloud_src.points.size (), cloud_tgt.points.size ());
    return;
  }
  if (cloud_src.points.size () < 4)     // need at least 4 samples
  {
    ROS_ERROR ("[pcl::IterativeClosestPointNonLinear::estimateRigidTransformationLM] Need at least 4 points to estimate a transform! Source and target have %zu points!", cloud_src.points.size ());
    return;
  }

  int m = cloud_src.points.size ();
  double *fvec = new double[m];
  int *iwa = new int[n_unknowns];
  int lwa = m * n_unknowns + 5 * n_unknowns + m;
  double *wa = new double[lwa];

  // Set the initial solution
  double *x = new double[n_unknowns];
  // Translation estimates - initial guess
  x[0] = 0; x[1] = 0; x[2] = 0;
  // Rotation estimates - initial guess quaternion: x-y-z-w
  x[3] = 0; x[4] = 0; x[5] = 0;

  // Set temporary pointers
  tmp_src_ = &cloud_src;
  tmp_tgt_ = &cloud_tgt;

  // Set tol to the square root of the machine. Unless high solutions are required, these are the recommended settings.
  double tol = sqrt (dpmpar (1));

  // Optimize using forward-difference approximation LM
  //int info = lmdif1 (&pcl::IterativeClosestPointNonLinear<PointSource, PointTarget>::functionToOptimize, this, m, n_unknowns, x, fvec, tol, iwa, wa, lwa);
  int info = lmdif1 (&pcl::IterativeClosestPointNonLinear<PointSource, PointTarget>::functionToOptimize, this, m, n_unknowns, x, fvec, tol, iwa, wa, lwa);

  // Compute the norm of the residuals
  ROS_DEBUG ("[pcl::%s::estimateRigidTransformationLM] LM solver finished with exit code %i, having a residual norm of %g. ",
             //"\nFinal solution: [%f %f %f %f] [%f %f %f]", 
             getClassName ().c_str (), info, enorm (m, fvec));
             //x[0], x[1], x[2], x[3], x[4], x[5], x[6]);

  delete [] wa; delete [] fvec;
  // <cloud_src,cloud_src> is the source dataset
  transformation_matrix.setZero ();

  // Return the correct transformation
  // Compute w from the unit quaternion
  Eigen::Quaternionf q (0, x[3], x[4], x[5]);
  q.w () = sqrt (1 - q.dot (q));
  transformation_matrix.topLeftCorner<3, 3> () = q.toRotationMatrix ();

  Eigen::Vector4f t (x[4], x[5], x[6], 1.0);
  transformation_matrix.block <4, 1> (0, 3) = t;

  tmp_src_ = tmp_tgt_ = NULL;

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
template <typename PointSource, typename PointTarget> inline int
  pcl::IterativeClosestPointNonLinear<PointSource, PointTarget>::functionToOptimize (void *p, int m, int n, const double *x, double *fvec, int iflag)
{
  IterativeClosestPointNonLinear *model = (IterativeClosestPointNonLinear*)p;

  // Copy the rotation and translation components
  Eigen::Vector4f t (x[0], x[1], x[2], 1.0);
  // Compute w from the unit quaternion
  Eigen::Quaternionf q (0, x[3], x[4], x[5]);
  q.w () = sqrt (1 - q.dot (q));
  // If the quaternion is used to rotate several points (>1) then it is much more efficient to first convert it
  // to a 3x3 Matrix. Comparison of the operation cost for n transformations:
  // * Quaternion: 30n
  // * Via a Matrix3: 24 + 15n
  Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Zero ();
  transformation_matrix.topLeftCorner<3, 3> () = q.toRotationMatrix ();
  transformation_matrix.block <4, 1> (0, 3) = t;

  double sigma = model->getMaxCorrespondenceDistance ();
  for (int i = 0; i < m; ++i)
  {
    // The last coordinate, p_src[3] is guaranteed to be set to 1.0 in registration.hpp
    Vector4fMapConst p_src = model->tmp_src_->points[i].getVector4fMap ();
    // The last coordinate, p_tgt[3] is guaranteed to be set to 1.0 in registration.hpp
    Vector4fMapConst p_tgt = model->tmp_tgt_->points[i].getVector4fMap ();

    Eigen::Vector4f pp = transformation_matrix * p_src;

    // Estimate the distance (cost function)
    //fvec[i] = model->distL2Sqr (p_tgt, pp);
    //fvec[i] = model->distL1 (pp, p_tgt);
    fvec[i] = model->distHuber (pp, p_tgt, sigma);
  }
  return (0);
}

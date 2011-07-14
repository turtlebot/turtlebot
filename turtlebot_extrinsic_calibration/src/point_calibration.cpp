#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry> 
#include <Eigen/StdVector>

#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>

#include <cminpack.h>

using namespace Eigen;
using namespace std;

typedef std::vector<Eigen::Vector3f,Eigen::aligned_allocator<Eigen::Vector3f> > PointVector;

const double pi = std::acos(-1.0);

class ObjectPose
{
  public:
    ObjectPose() : translation(0,0,0), orientation(1,0,0,0) { }
    ObjectPose(Vector3f trans, Quaternionf orient): translation(trans), orientation(orient) { }
    ObjectPose(Quaternionf orient) : translation(0,0,0),  orientation(orient) { }
    ObjectPose(Vector3f trans) : translation(trans), orientation(1,0,0,0) { }
    
    Vector3f translation;
    Quaternionf orientation;
    
    Transform<float,3,Affine> transform()
    {
      Transform<float,3,Affine> newtrans(orientation);
      newtrans.pretranslate(translation);
      
      return newtrans;
    }
};

class CalibrationWorld
{
  public:
    CalibrationWorld();
    void reprojectPoints();
    void updateBasePos(Vector3f trans, Quaternionf orient);
    void getState(ObjectPose& base_pose_out, PointVector& kinect_points);
    
  public:
  //private:
    ObjectPose base_pose;
    Transform<float,3,Affine> base_kinect; // Should *this* be a transform?
    ObjectPose kinect;
    ObjectPose target;
    
    // How to best represent points?
    // Vector of 3f positions?
    PointVector true_points; // Points on the target to project
    PointVector projected_points; // Points on the target projected into the world frame
    PointVector kinect_points; // Points on the target projected into the kinect frame
};

class EstimateKinectTransform
{
  public:
    EstimateKinectTransform() : result_transform(Translation3f(0,0,0)) {}
  
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

CalibrationWorld::CalibrationWorld()
  : base_pose(), base_kinect(Translation3f(-1, 0, 0)), target(Vector3f(2, 0, 0))
  
  //: base_pose(), base_kinect(AngleAxis<float>(pi/2.0, Vector3f(0,0,1))), target(Vector3f(2, 0, 0))
{
  // Set positions of objects in the world.
  true_points.push_back(Vector3f(0,0,0));
  true_points.push_back(Vector3f(0,1,0));
  true_points.push_back(Vector3f(0,0,1));
  
  projected_points.resize(true_points.size());
  kinect_points.resize(true_points.size());
}

void CalibrationWorld::reprojectPoints()
{
  Transform<float,3,Affine> kinecttransform = base_pose.transform()*base_kinect;
  
  //cout << "Base transform: " << endl << base_pose.transform().matrix() << endl;
  //cout << "Kinect transform: " << endl << kinecttransform.matrix() << endl;
  
  //cout << "Target Rotation: " << target.orientation.coeffs().transpose() << " Translation: " << target.translation.transpose() << endl;
  for (unsigned int i=0; i < true_points.size(); i++)
  {
    //cout << "True point: " << true_points[i].transpose() << endl;
    projected_points[i] = target.orientation.toRotationMatrix()*true_points[i] + target.translation;
    //cout << "World point: " << projected_points[i].transpose() << endl;
    kinect_points[i] = kinecttransform.inverse()*projected_points[i];
    //cout << "Kinect point: " << kinect_points[i].transpose() << endl;
    
    Vector3f backcalcpoint = kinecttransform*kinect_points[i];
    //cout << "Back-calc world point: " << backcalcpoint.transpose() << endl;
  }
  
  // Tentatively going to say that this seems right.
}

void CalibrationWorld::updateBasePos(Vector3f trans, Quaternionf orient)
{
  base_pose.translation = trans;
  base_pose.orientation = orient;
}

double EstimateKinectTransform::computeError(int m, Transform<float, 3, Affine> base_kinect)
{
  if (base_pose.size() < m+2)
    return 0;
  
  // Get the transforms and points for this iteration.
  Transform<float,3,Affine> kinect_transform = base_pose[m+1].transform()*base_kinect;
  PointVector kinect_points = target_points[m+1];
  
  Transform<float,3,Affine> previous_transform = base_pose[m].transform()*base_kinect;
  PointVector previous_points = target_points[m];
  
  double error = 0;
  for (unsigned int i=0; i < kinect_points.size(); i++)
  { 
    Vector3f previous_point_world = previous_transform*previous_points[i];
    //cout << "Previous world point: " << previous_point_world.transpose() << endl;
    
    Vector3f current_point_world = kinect_transform*kinect_points[i];
    //cout << "Current world point: " << current_point_world.transpose() << endl;
    
    error += reprojectionError(previous_point_world, current_point_world);
  }
  
  return error;
}

double EstimateKinectTransform::reprojectionError(Vector3f point_obs, Vector3f point_calc)
{
  // Squared distance between 2 vectors is the squared norm of their difference.
  return (point_obs - point_calc).squaredNorm();
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

void EstimateKinectTransform::computeTransform()
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
  x[0] = 0; x[1] = 0; x[2] = 0;
  // Rotation estimates - initial guess quaternion: x-y-z-w
  x[3] = 0; x[4] = 0; x[5] = 0;

  // Set tol to the square root of the machine. Unless high solutions are required, these are the recommended settings.
  double tol = 1e-4;//sqrt (dpmpar (1));

  double precost = computeTotalCost();
  cout << "About to run LM. Total pre-cost: " << precost/m << endl;
  // Optimize using forward-difference approximation LM
  int info = lmdif1 (&EstimateKinectTransform::functionToOptimize, this, m, n_unknowns, x, fvec, tol, iwa, wa, lwa); // Function call

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
  
  cout << "Total cost at step: " << totalcost << endl;
  return (0);
}


int main( int argc, char** argv )
{
  CalibrationWorld world;
  EstimateKinectTransform est;
  
  world.reprojectPoints();
  est.addData(world.base_pose, world.kinect_points);
  cout << endl << "Total cost: " << est.computeTotalCost() << endl << endl;

  world.updateBasePos(Vector3f(1,0,0), Quaternionf(1,0,0,0));
  world.reprojectPoints();
  est.addData(world.base_pose, world.kinect_points);
  cout << endl << "Total cost: " << est.computeTotalCost() << endl << endl;
  
  world.updateBasePos(Vector3f(1,0,0), Quaternionf(AngleAxis<float>(pi/2.0, Vector3f(0,0,1))));
  world.reprojectPoints();
  est.addData(world.base_pose, world.kinect_points);
  cout << endl << "Total cost: " << est.computeTotalCost() << endl << endl;
  
  world.updateBasePos(Vector3f(0,1,0), Quaternionf(AngleAxis<float>(0, Vector3f(0,0,1))));
  world.reprojectPoints();
  est.addData(world.base_pose, world.kinect_points);
  cout << endl << "Total cost: " << est.computeTotalCost() << endl << endl;
  
  world.updateBasePos(Vector3f(1,1,0), Quaternionf(AngleAxis<float>(pi/2.0, Vector3f(0,0,1))));
  world.reprojectPoints();
  est.addData(world.base_pose, world.kinect_points);
  cout << endl << "Total cost: " << est.computeTotalCost() << endl << endl;
  
  world.updateBasePos(Vector3f(3,0,0), Quaternionf(AngleAxis<float>(pi/1.0, Vector3f(0,0,1))));
  world.reprojectPoints();
  est.addData(world.base_pose, world.kinect_points);
  cout << endl << "Total cost: " << est.computeTotalCost() << endl << endl;
  
  world.updateBasePos(Vector3f(3,3,0), Quaternionf(AngleAxis<float>(pi/1.0, Vector3f(0,0,1))));
  world.reprojectPoints();
  est.addData(world.base_pose, world.kinect_points);
  cout << endl << "Total cost: " << est.computeTotalCost() << endl << endl;
  
  est.result_transform.translation() = Vector3f(-.05, 0, 0);
  cout << endl << "Total cost: " << est.computeTotalCost() << endl << endl;
  
  est.result_transform.translation() = Vector3f(-.1, 0, 0);
  cout << endl << "Total cost: " << est.computeTotalCost() << endl << endl;
  
  est.computeTransform();
  cout << endl << "Final transform: " << endl << est.getTransform().matrix() << endl;
}


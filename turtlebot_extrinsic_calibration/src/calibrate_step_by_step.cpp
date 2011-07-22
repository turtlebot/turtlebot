// Calibrate step-by-step

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <pcl_ros/point_cloud.h>

#include <turtlebot_extrinsic_calibration/estimate_kinect_link.h>
#include <turtlebot_extrinsic_calibration/detect_calibration_pattern.h>
#include <turtlebot_extrinsic_calibration/detect_ground_plane.h>

using namespace std;
using namespace Eigen;

const std::string world_frame = "/odom_combined";
const std::string base_frame = "/base_link";
const std::string camera_frame = "/kinect_rgb_optical_frame";

const std::string camera_topic = "/camera/rgb/";
const std::string image_topic = "image_mono";

const int checkerboard_width = 6;
const int checkerboard_height = 7;
const double checkerboard_grid = 0.027;

enum CalibrationState
{
  CALIB_INIT, CALIB_ROTATION, CALIB_GROUND_PLANE, CALIB_XY, CALIB_FINISHED 
};

class CalibrateStepByStep
{
  public:
    // Nodes and publishers/subscribers
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Publisher pub_;
  
    // Image and camera info subscribers;
    ros::Subscriber image_sub_; 
    ros::Subscriber info_sub_;
    ros::Subscriber pointcloud_sub_;

    // Structures for interacting with ROS messages
    cv_bridge::CvImagePtr bridge_;
    tf::TransformListener tf_listener_;
    image_geometry::PinholeCameraModel cam_model_;
    
    // Calibration objects
    PatternDetector pattern_detector_;
    EstimateKinectTransform est_;
    GroundPlaneDetector ground_plane_detector_;
    
    // State variable
    CalibrationState state_;
    
    // Order of states
    std::vector<CalibrationState> state_list_;
    int state_index_;
    
    // The optimized transform
    Eigen::Transform<float, 3, Eigen::Affine> transform;
    
    // Parameters for image callback
    // (Look into encapsulating these...)
    PointVector true_points_;
    Eigen::Vector3f translation_old_;
    Eigen::Quaternionf orientation_old_;
    
    int iterations;


public:
  CalibrateStepByStep()
    : nh_(), it_(nh_), state_(CALIB_INIT), state_index_(0)
  {
    pattern_detector_.setPattern(cv::Size(checkerboard_width, checkerboard_height), checkerboard_grid, CHESSBOARD);
    //true_points_.push_back(Eigen::Vector3f(checkerboard_width*checkerboard_grid,0,0));
    true_points_.push_back(Eigen::Vector3f(0,0,0));
    //true_points_.push_back(Eigen::Vector3f(0,checkerboard_height*checkerboard_grid,0));
    
    translation_old_.setZero();
    orientation_old_.setIdentity();
    
    setState(CALIB_INIT);
    
    transform.translation().setZero();
    transform.matrix().topLeftCorner<3, 3>() = Quaternionf().setIdentity().toRotationMatrix();
    
    //transform.matrix().topLeftCorner<3, 3>() = Quaternionf(.5,-.5,.5,-.5).toRotationMatrix();
    
    state_list_.push_back(CALIB_INIT);
    state_list_.push_back(CALIB_ROTATION);
    //state_list_.push_back(CALIB_GROUND_PLANE);
    state_list_.push_back(CALIB_XY);
    state_list_.push_back(CALIB_FINISHED);
    
    // DEBUG
    pub_ = it_.advertise("image_out", 1);
  }
    
  void advanceState()
  {
    unsetState();
    state_index_++;
    setState(state_list_[state_index_]);
  }
  
  void setState(CalibrationState new_state)
  {
    state_ = new_state;
  
    displayResults();
  
    switch (new_state)
    {
      case CALIB_INIT:
        info_sub_ = nh_.subscribe(camera_topic + "camera_info", 1, &CalibrateStepByStep::infoCallback, this);
        cout << "[calibrate] Initialized." << endl;
        break;
      case CALIB_ROTATION:
        pointcloud_sub_ = nh_.subscribe<pcl::PointCloud<pcl::PointXYZRGB> >
                                  (camera_topic + "points", 1, &CalibrateStepByStep::pointcloudToImageCallback, this);
        //image_sub_ = nh_.subscribe(camera_topic + image_topic, 1, &CalibrateStepByStep::imageCallback, this);
        iterations = 0;
        cout << "[calibrate] Calibrating rotation." << endl;
        est_.setMask(ESTIMATOR_MASK_ROTATION);
        break;
      case CALIB_GROUND_PLANE:
        pointcloud_sub_ = nh_.subscribe<pcl::PointCloud<pcl::PointXYZ> >
                                  (camera_topic + "points", 1, &CalibrateStepByStep::pointcloudCallback, this);
        cout << "[calibrate] Calibrating ground plane." << endl;
        break;
      case CALIB_XY:
        pointcloud_sub_ = nh_.subscribe<pcl::PointCloud<pcl::PointXYZRGB> >
                                  (camera_topic + "points", 1, &CalibrateStepByStep::pointcloudToImageCallback, this);
        //image_sub_ = nh_.subscribe(camera_topic + image_topic, 1, &CalibrateStepByStep::imageCallback, this);
        iterations = 0;
        est_.setMask(ESTIMATOR_MASK_XY);
        cout << "[calibrate] Calibrating x-y." << endl;
        break;
      case CALIB_FINISHED:
        displayResults();
        ros::shutdown();
        break;
      default:
        ROS_WARN("[calibrate] Unknown state!");
        break;
    }
  }
  
  void unsetState()
  {
    switch (state_)
    {
      case CALIB_INIT:
        info_sub_.shutdown();
        break;
      case CALIB_ROTATION:
        pointcloud_sub_.shutdown();
        //image_sub_.shutdown();
        break;
      case CALIB_GROUND_PLANE:
        pointcloud_sub_.shutdown();
        break;
      case CALIB_XY:
        pointcloud_sub_.shutdown();
        //image_sub_.shutdown();
        break;
      case CALIB_FINISHED:
        break;
      default:
        ROS_WARN("[calibrate] Unknown state!");
        break;
    }
  
  }
  
  void infoCallback(const sensor_msgs::CameraInfoConstPtr& info_msg)
  {
    cam_model_.fromCameraInfo(info_msg);
    
    cv::Mat K = (cv::Mat_<double>(3,3) << 
        538.04668883179033, 0.0, 333.47627148365939, 
        0.0, 533.58807413837224, 261.72627024909525, 
        0.0, 0.0, 1.0);
    cv::Mat D = (cv::Mat_<double>(4,1) << 
        0.11670698134925221, -0.21651116066856213, -0.0028579589456466377, 
        0.011540507063821316);

    //pattern_detector_.setCameraMatrices(K, D);
    pattern_detector_.setCameraMatrices(cam_model_.intrinsicMatrix(), cam_model_.distortionCoeffs());
    advanceState();
  }
  
  void pointcloudCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg)
  {
    // Figure out normal...
    // How do you figure out a normal from a quaternion?
    // Should be R*(0,0,1)
    // Maybe R*(0,0,-1)
    // Maybe R.transpose()
    // WHO KNOWS
    Vector3f normal = transform.rotation()*Vector3f(1,0,0);
    
    cout << "Normal we're looking for: " << normal << endl;
    
    ground_plane_detector_.setExpectedNormal(normal);
    
    if (ground_plane_detector_.findGroundPlane(msg))
    {
      // Is this actually z OH GOD I DON'T KNOW
      // This is probably horribly wrong
      transform.translation().z() = -ground_plane_detector_.getDistance();
      advanceState();
    }
  }
  
  void pointcloudToImageCallback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& msg)
  {
    const sensor_msgs::ImagePtr image(new sensor_msgs::Image);
    pcl::toROSMsg(*msg, *image);
    imageCallback(image);
  }
  
  
  void imageCallback(const sensor_msgs::ImageConstPtr& image_msg)
  {
    try
    {
      bridge_ = cv_bridge::toCvCopy(image_msg, "mono8");
    }
    catch (cv_bridge::Exception& ex)
    {
      ROS_ERROR("[calibrate] Failed to convert image");
      return;
    }

    Eigen::Vector3f translation;
    Eigen::Quaternionf orientation;
    
    if (!pattern_detector_.detectPattern(bridge_->image, translation, orientation))
      return;
    
    tf::StampedTransform base_transform;
    try
    {
      ros::Time acquisition_time = image_msg->header.stamp;
      ros::Duration timeout(1.0 / 30.0);
      
      // Get base_link transform
      tf_listener_.waitForTransform(world_frame, base_frame,
                                    acquisition_time, timeout);
      tf_listener_.lookupTransform(world_frame, base_frame,
                                   acquisition_time, base_transform);
    }
    catch (tf::TransformException& ex)
    {
      ROS_WARN("[calibrate] TF exception:\n%s", ex.what());
      return;
    }
    
    // Alright, now we have a btTransform...
    // Convert it to something we can use.
    // ObjectPose? Eigen::Transform?
    Eigen::Vector3f base_translation(base_transform.getOrigin().x(), 
                  base_transform.getOrigin().y(), base_transform.getOrigin().z());
    Eigen::Quaternionf base_orientation(base_transform.getRotation().w(), 
                  base_transform.getRotation().x(), base_transform.getRotation().y(), 
                  base_transform.getRotation().z());
    
    // Enough of a change to do keyframing
    if ((translation - translation_old_).norm() > 0.01 || (orientation.angularDistance(orientation_old_)) > 0.01)
    { 
      PointVector projected_points(true_points_.size());
      
      for (unsigned int i=0; i < true_points_.size(); i++)
      {
        projected_points[i] = orientation.toRotationMatrix()*true_points_[i] + translation;
      } 
      
      est_.addData(ObjectPose(base_translation, base_orientation), projected_points);
      
      iterations++;
      
      translation_old_ = translation;
      orientation_old_ = orientation;
      
      // DEBUG
      pub_.publish(bridge_->toImageMsg());
      
      if (iterations >= 30)
      {
        cout << endl << "Total cost with calibration: " << est_.computeTotalCost() << endl;
              
        est_.computeTransform(transform);
        cout << endl << "Final transform with " << est_.base_pose.size() <<  " poses: " << endl
              << "Translation: [" << est_.getTransform().translation().transpose() 
              << "] \n Rotation (Quat): ["  << Quaternionf(est_.getTransform().rotation()).coeffs().transpose()<< "]" << endl;
        
        transform = est_.getTransform();
        advanceState();
      }
    }
  }
  
  void displayResults()
  {
    tf::StampedTransform kinect_transform;
    ros::Time acquisition_time = ros::Time::now();
    ros::Duration timeout(10.0 / 30.0);
    try 
    {
      // Get the original kinect_link transform                             
      tf_listener_.waitForTransform(base_frame, camera_frame,
                                    ros::Time(0), timeout);
      tf_listener_.lookupTransform(base_frame, camera_frame,
                                   ros::Time(0), kinect_transform);
    }
    catch (tf::TransformException& ex) 
    {
      ROS_WARN("[calibrate] TF exception:\n%s", ex.what());
      return;
    }
    
    Eigen::Vector3f kinect_translation(kinect_transform.getOrigin().x(), 
            kinect_transform.getOrigin().y(), kinect_transform.getOrigin().z());
    Eigen::Quaternionf kinect_orientation(kinect_transform.getRotation().w(), 
            kinect_transform.getRotation().x(), kinect_transform.getRotation().y(), 
            kinect_transform.getRotation().z());
            
    cout << endl << "Final transform: " << endl
        << "Translation: [" << transform.translation().transpose() 
        << "] \n Rotation (Quat): ["  << Quaternionf(transform.rotation()).coeffs().transpose() << "]" << endl;
        
    cout << endl << "Compared to (TF): " << endl
        << "Translation: [" << kinect_translation.transpose() 
        << "] \n Rotation (Quat): [" << kinect_orientation.coeffs().transpose() << "]" << endl;
  }
  
};

// Prep
//  - Subscribe to camera_info
//  - Subscribe to image topic
//  - Create tf_listener


// Step 1: Optimize just the rotations from checkerboard (theta, phi, the other one)
//  - Create a transform from checkerboard object
//  - Connect its callback up
//  - Do I *need* a transform from checkerboard object?
//  - Create a estimate transform object
//    - Set a mask on x to just do rotations
//  - Connect callback up
//  - Check for convergence

// Step 2: Find the ground plane (z)
//  - Create a ground plane detector
//  - Connect callback up
//  - Check for consistency of ground planes

// Step 3: Optimize just the remaining parameters (xy)
//  - Create a new estimate transform object or use the same one
//  - Mask x for just xy optimization
//  - Connect callback up
//  - Check for convergence


// Step 4: Output and visualize



// Step 5: Update URDF



int main(int argc, char** argv)
{
  ros::init(argc, argv, "calibrate_step_by_step");
  
  CalibrateStepByStep cal;
  ros::spin();
}


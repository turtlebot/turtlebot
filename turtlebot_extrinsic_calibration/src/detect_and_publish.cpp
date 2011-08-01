#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>



#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <turtlebot_extrinsic_calibration/detect_calibration_pattern.h>
#include <turtlebot_extrinsic_calibration/detect_ground_plane.h>

using namespace std;
using namespace Eigen;

const std::string world_frame = "/odom_combined";
const std::string base_frame = "/base_link";
const std::string camera_frame = "/openni_rgb_optical_frame";
const std::string target_frame = "/target_frame";
const std::string ground_frame = "/ground_frame";

const std::string camera_topic = "/watcher_kinect/camera/rgb/";
const std::string image_topic = camera_topic + "image_mono";
const std::string cloud_topic = camera_topic + "points";
const std::string info_topic = camera_topic + "camera_info";

const int checkerboard_width = 6;
const int checkerboard_height = 7;
const double checkerboard_grid = 0.027;

const int circles_width = 3;
const int circles_height = 5;
const double circles_grid = 0.04;

const double pi = 3.141592653589793238462643383279502;

class DetectAndPublish
{
  public:
    // Nodes and publishers/subscribers
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Publisher pub_;
    ros::Publisher cloud_pub_;
    ros::Publisher detector_pub_;
    
    // Image and camera info subscribers;
    ros::Subscriber image_sub_; 
    ros::Subscriber info_sub_;
    ros::Subscriber pointcloud_sub_;

    // Structures for interacting with ROS messages
    cv_bridge::CvImagePtr bridge_;
    tf::TransformListener tf_listener_;
    tf::TransformBroadcaster tf_broadcaster_;
    image_geometry::PinholeCameraModel cam_model_;
    
    // Calibration objects
    PatternDetector pattern_detector_;
    GroundPlaneDetector ground_plane_detector_;
    
    // Visualization for markers
    pcl::PointCloud<pcl::PointXYZ> detector_points_;
    
    // Transform to ground
    tf::Transform ground_plane_transform;
    
    bool calibrated;
    
public:
    DetectAndPublish()
    : nh_(), it_(nh_), calibrated(false)
  {
    //pattern_detector_.setPattern(cv::Size(checkerboard_width, checkerboard_height), checkerboard_grid, CHESSBOARD);
    pattern_detector_.setPattern(cv::Size(circles_width, circles_height), circles_grid, ASYMMETRIC_CIRCLES_GRID);
    
    ground_plane_transform.setOrigin(tf::Vector3(0,0,0));
    ground_plane_transform.setRotation(tf::Quaternion(0,0,0,1));
    
    pub_ = it_.advertise("watcher_image_out", 1);
    cloud_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ> >("watcher_ground_cloud", 1, true);
    detector_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ> >("detector_cloud", 1);
    info_sub_ = nh_.subscribe(info_topic, 1, &DetectAndPublish::infoCallback, this);
    pointcloud_sub_ = nh_.subscribe<pcl::PointCloud<pcl::PointXYZ> >
                                  (cloud_topic, 1, &DetectAndPublish::pointcloudCallback, this);
                                  
    convertIdealPointstoPointcloud();
  }
    
  void infoCallback(const sensor_msgs::CameraInfoConstPtr& info_msg)
  {
    if (!calibrated)
    {
      cam_model_.fromCameraInfo(info_msg);
      
      cv::Mat K = (cv::Mat_<double>(3,3) << 
          512.74366224945743, 0.0, 308.8776500933655, 
          0.0, 513.03387431919771, 254.88934114645886, 
          0.0, 0.0, 1.0);
      cv::Mat D = (cv::Mat_<double>(4,1) << 
          0.17558166926872085, -0.34277921226861113, 0.0069688595545382486, 
          -0.0056884546815529954, 0.20186938060741913);

      //pattern_detector_.setCameraMatrices(K, D);
      pattern_detector_.setCameraMatrices(cam_model_.intrinsicMatrix(), cam_model_.distortionCoeffs());
      
      calibrated = true;
      image_sub_ = nh_.subscribe(image_topic, 1, &DetectAndPublish::imageCallback, this);
      
      cout << "Got info!" << endl;
    }
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
    
    
    tf::Transform transform, shifted_transform, shift;
    Eigen::Quaternionf shift_orientation(Eigen::AngleAxisf(-pi/2, Eigen::Vector3f(0,0,1)));
    try
    {
      transform.setOrigin( tf::Vector3(translation.x(), translation.y(), translation.z()) );
      transform.setRotation( tf::Quaternion(orientation.x(), orientation.y(), orientation.z(), orientation.w()) );
      
      shift.setOrigin( tf::Vector3(0.08, 0.08, 0) );
      shift.setRotation( tf::Quaternion(shift_orientation.x(), shift_orientation.y(), shift_orientation.z(), shift_orientation.w() ));
      
      tf_broadcaster_.sendTransform(tf::StampedTransform(transform*shift, image_msg->header.stamp, image_msg->header.frame_id, target_frame));
      
      tf_broadcaster_.sendTransform(tf::StampedTransform(ground_plane_transform, image_msg->header.stamp, image_msg->header.frame_id, ground_frame));
    }
    catch (tf::TransformException& ex)
    {
      ROS_WARN("[calibrate] TF exception:\n%s", ex.what());
      return; 
    }
    
    // Display to rviz
    pcl::PointCloud<pcl::PointXYZ> transformed_detector_points;
    
    pcl_ros::transformPointCloud(detector_points_, transformed_detector_points, transform);
    
    transformed_detector_points.header.frame_id = image_msg->header.frame_id;
    detector_pub_.publish(transformed_detector_points);
    
    pub_.publish(bridge_->toImageMsg());
  }
    
  void pointcloudCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg)
  {
    Vector3f normal = Vector3f(1,1,1);
    
    cout << "Normal we're looking for: " << normal << endl;
    
    ground_plane_detector_.setExpectedNormal(normal);
    
    if (ground_plane_detector_.findGroundPlane(msg))
    {
      cout << "Ground plane distance: " << ground_plane_detector_.getDistance() << endl << endl;
      cloud_pub_.publish(ground_plane_detector_.cloud_output);
    }
    
    
    Eigen::Quaternionf quat;
    quat.setFromTwoVectors(Vector3f(0,0,-1), ground_plane_detector_.getNormal());
    ground_plane_transform.setRotation( tf::Quaternion(quat.x(), quat.y(), quat.z(), quat.w()) );
    ground_plane_transform.setOrigin( tf::Vector3(0,0,0) );
    ground_plane_transform.setOrigin( ground_plane_transform*tf::Vector3(0, 0, ground_plane_detector_.getDistance()  ) );
    //ground_plane_transform = ground_plane_transform.inverse();
    
    pointcloud_sub_.shutdown();
  }
    
    
  void convertIdealPointstoPointcloud()
  {
    detector_points_.points.resize(pattern_detector_.ideal_points.size());
    for (unsigned int i=0; i < pattern_detector_.ideal_points.size(); i++)
    {
      cv::Point3f pt = pattern_detector_.ideal_points[i];
      detector_points_[i].x = pt.x; detector_points_[i].y = pt.y; detector_points_[i].z = pt.z; 
    }
  }
    
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "detect_and_publish");
  
  DetectAndPublish detect;
  ros::spin();
}

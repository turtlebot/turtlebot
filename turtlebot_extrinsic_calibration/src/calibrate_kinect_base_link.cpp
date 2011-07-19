#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <visualization_msgs/Marker.h>

#include <turtlebot_extrinsic_calibration/estimate_kinect_link.h>


// Fix the shit out of this
#include "detect_calibration_pattern.cpp"

using namespace std;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo> CameraSyncPolicy;

class CalibrateExtrinsics
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Publisher pub_;

  //image_transport::Subscriber image_sub_; 
  //image_transport::SubscriberFilter image_sub_; 
  
  message_filters::Subscriber<sensor_msgs::Image> image_sub_; 
  message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub_;
  ros::Subscriber info_sub_2;
  message_filters::Synchronizer<CameraSyncPolicy> sync_;

  image_transport::Subscriber im_sub_;
  
  ros::Publisher marker_pub_;
  
  cv_bridge::CvImagePtr bridge_;
  tf::TransformListener tf_listener_;
  //sensor_msgs::CvBridge bridge_;
  image_geometry::PinholeCameraModel cam_model_;
  //std::vector<std::string> frame_ids_;
  //CvFont font_;
  
  PatternDetector pattern_detector_;
  EstimateKinectTransform est_;
  
  PointVector true_points_;

public:
  CalibrateExtrinsics()
    : nh_(), it_(nh_), 
        image_sub_ (nh_, "/camera/rgb/image_mono", 3),
        info_sub_(nh_, "/camera/rgb/camera_info", 3),
        sync_(CameraSyncPolicy(10), image_sub_, info_sub_)
  {
    //std::string image_topic = nh_.resolveName("image");
    //im_sub_ = it_.subscribe("/camera/rgb/image_color", 1, &CalibrateExtrinsics::imageCb, this);
    
    //image_sub_.subscribe(it_, "/wide_stereo/left/image_mono", 10);
    //info_sub_.subscribe(nh_, "/wide_stereo/left/camera_info", 10);
    // image_sub_.subscribe(it_, "/camera/rgb/image_color", 10);
    // info_sub_.subscribe(nh_, "/camera/rgb/camera_info", 10);
    //sync_.connectInput(image_sub_, info_sub_);
    sync_.registerCallback(boost::bind(&CalibrateExtrinsics::imageCb, this, _1, _2));
    
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("debug_markers", 10);
    
    
    
    pub_ = it_.advertise("image_out", 1);
    //cvInitFont(&font_, CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5);
    
    pattern_detector_.setPattern(cv::Size(6, 7), 0.027, CHESSBOARD);
    
    
    true_points_.push_back(Eigen::Vector3f(0,0,0));
    true_points_.push_back(Eigen::Vector3f(1,0,0));
    true_points_.push_back(Eigen::Vector3f(0,1,0));
    
    
    cout << "Finished constructor." << endl;
  }

  void testCb(const sensor_msgs::CameraInfoConstPtr& info_msg)
  {
    cout << "Got camera info!" << endl;
  }

  void imageCb(const sensor_msgs::ImageConstPtr& image_msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg)
  {
    ROS_INFO("Image callback! \n");
    //IplImage* image = NULL;
    try {
      bridge_ = cv_bridge::toCvCopy(image_msg, "mono8");
      
      //image = bridge_.imgMsgToCv(image_msg, "mono8");
    }
    catch (cv_bridge::Exception& ex) {
      ROS_ERROR("[calibrate] Failed to convert image");
      return;
    }

    cam_model_.fromCameraInfo(info_msg);
    
    pattern_detector_.setCameraMatrices(cam_model_.intrinsicMatrix(), cam_model_.distortionCoeffs());
    
    Eigen::Vector3f translation;
    Eigen::Quaternionf orientation;
    
    bool detected = pattern_detector_.detectPattern(bridge_->image, translation, orientation);
    
    if (detected)
    {
      cout << "Translation: " << translation.transpose() << endl
           << "Rotation: " << orientation.toRotationMatrix() << endl;
     
      // If pattern detected, should probably do stuff.
      // Visualize the markers in 3D: 
           
      publishTargetMarker(translation, orientation);

    }
    
    tf::StampedTransform transform;
    try {
      ros::Time acquisition_time = info_msg->header.stamp;
      ros::Duration timeout(1.0 / 30);
      tf_listener_.waitForTransform("/odom_combined", "/base_link",
                                    acquisition_time, timeout);
      tf_listener_.lookupTransform("/odom_combined", "/base_link",
                                   acquisition_time, transform);
    }
    catch (tf::TransformException& ex) {
      ROS_WARN("[calibrate] TF exception:\n%s", ex.what());
      return;
    }
    
    // Alright, now we have a btTransform...
    // Convert it to something we can use.
    // ObjectPose? Eigen::Transform?
    Eigen::Vector3f base_translation(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
    Eigen::Quaternionf base_orientation(transform.getRotation().w(), transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z());
    
    cout << endl << "Base Translation: " << base_translation.transpose() 
         << endl << "Base Quaternion: " << base_orientation.coeffs().transpose() << endl;
  
    /*tf::Point pt = transform.getOrigin();
    cv::Point3d pt_cv(pt.x(), pt.y(), pt.z());
    cv::Point2d uv;
    cam_model_.project3dToPixel(pt_cv, uv);

    static const int RADIUS = 3;
    cvCircle(image, uv, RADIUS, CV_RGB(255,0,0), -1);
    CvSize text_size;
    int baseline;
    cvGetTextSize(frame_id.c_str(), &font_, &text_size, &baseline);
    CvPoint origin = cvPoint(uv.x - text_size.width / 2,
                             uv.y - RADIUS - baseline - 3);
    cvPutText(image, frame_id.c_str(), origin, &font_, CV_RGB(255,0,0)); */
    
    pub_.publish(bridge_->toImageMsg());
    
    // Let's add this to our LM thing. And them optimize it. Yeaaaah!
    if (detected)
    {
      Eigen::Matrix3f rotmat;
      rotmat << 0, 0, 1, -1, 0, 0, 0, -1, 0;
      Eigen::Quaternionf quat(rotmat);
      
      translation = quat*translation;
      orientation = quat*orientation;
    
      PointVector projected_points(true_points_.size());
      for (unsigned int i=0; i < true_points_.size(); i++)
      {
        projected_points[i] = orientation.toRotationMatrix()*true_points_[i] + translation;
      } 
      
      est_.addData(ObjectPose(base_translation, base_orientation), projected_points);
      cout << endl << "Total cost: " << est_.computeTotalCost() << endl << endl;
      est_.computeTransform();
      cout << endl << "Final transform: " << endl << est_.getTransform().matrix() << endl;
    }
  }
  
  void publishTargetMarker(Eigen::Vector3f& trans_, Eigen::Quaternionf& orient_)
  {
    visualization_msgs::Marker target_marker;
    target_marker.header.frame_id = "/kinect_link";
    target_marker.header.stamp = ros::Time::now();
    
    target_marker.id = 0;
    target_marker.type = visualization_msgs::Marker::CUBE;
    
    target_marker.scale.x = 0.25;
    target_marker.scale.y = 0.5;
    target_marker.scale.z = 0.01;
    
    target_marker.color.r = 1.0f;
    target_marker.color.g = 1.0f;
    target_marker.color.b = 1.0f;
    target_marker.color.a = 1.0;
    
    Eigen::Matrix3f rotmat;
    rotmat << 0, 0, 1, -1, 0, 0, 0, -1, 0;
    Eigen::Quaternionf quat(rotmat);
    
    cout << "Rotation matrix: " << endl << rotmat << endl;
    
    Eigen::Vector3f translation = quat*trans_;
    Eigen::Quaternionf orientation = quat*orient_; 
    
    target_marker.pose.position.x = translation.x();
    target_marker.pose.position.y = translation.y();
    target_marker.pose.position.z = translation.z();
    
    target_marker.pose.orientation.x = orientation.x();
    target_marker.pose.orientation.y = orientation.y();
    target_marker.pose.orientation.z = orientation.z();
    target_marker.pose.orientation.w = orientation.w();
    

    
    marker_pub_.publish(target_marker);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "calibrate_kinect_base");
  
  CalibrateExtrinsics cal;
  ros::spin();
}


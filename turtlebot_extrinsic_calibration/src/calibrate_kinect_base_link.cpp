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
#include <geometry_msgs/PoseArray.h>

#include <turtlebot_extrinsic_calibration/estimate_kinect_link.h>
#include <turtlebot_extrinsic_calibration/detect_calibration_pattern.h>

using namespace std;
using namespace Eigen;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo> CameraSyncPolicy;

const std::string world_frame = "/odom_combined";
const std::string base_frame = "/base_link";
const std::string camera_frame = "/kinect_rgb_optical_frame";

const std::string camera_topic = "/camera/rgb/";

class CalibrateExtrinsics
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Publisher pub_;

  //image_transport::Subscriber image_sub_; 
  //image_transport::SubscriberFilter image_sub_; 
  
  //message_filters::Subscriber<sensor_msgs::Image> image_sub_; 
  //message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub_;
  //message_filters::Synchronizer<CameraSyncPolicy> sync_;
  
  
  ros::Subscriber image_sub_; 
  ros::Subscriber info_sub_;
  
  ros::Publisher marker_pub_;
  ros::Publisher marker_pose_pub_;
  ros::Publisher kinect_pose_pub_;
  ros::Publisher calibrated_pose_pub_;
  
  cv_bridge::CvImagePtr bridge_;
  tf::TransformListener tf_listener_;
  image_geometry::PinholeCameraModel cam_model_;
  
  // Currently unused
  sensor_msgs::CameraInfo cam_info_;
  
  PatternDetector pattern_detector_;
  EstimateKinectTransform est_;
  
  PointVector true_points_;
  Eigen::Vector3f translation_old_;
  Eigen::Quaternionf orientation_old_;
  
  bool calibrated;

public:
  CalibrateExtrinsics()
    : nh_(), it_(nh_), calibrated(false)
        //image_sub_ (nh_, "/camera/rgb/image_mono", 3),
        //info_sub_(nh_, "/camera/rgb/camera_info", 3),
        //sync_(CameraSyncPolicy(10), image_sub_, info_sub_)
  {
    //std::string image_topic = nh_.resolveName("image");
    //im_sub_ = it_.subscribe("/camera/rgb/image_color", 1, &CalibrateExtrinsics::imageCb, this);
    
    //image_sub_.subscribe(it_, "/wide_stereo/left/image_mono", 10);
    //info_sub_.subscribe(nh_, "/wide_stereo/left/camera_info", 10);
    // image_sub_.subscribe(it_, "/camera/rgb/image_color", 10);
    // info_sub_.subscribe(nh_, "/camera/rgb/camera_info", 10);
    //sync_.connectInput(image_sub_, info_sub_);
    
    
    //sync_.registerCallback(boost::bind(&CalibrateExtrinsics::imageCb, this, _1, _2));
    
    info_sub_ = nh_.subscribe(camera_topic + "camera_info", 1, &CalibrateExtrinsics::infoCb, this);
    image_sub_ = nh_.subscribe(camera_topic + "image_color", 1, &CalibrateExtrinsics::imageCb, this);
    
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("debug_markers", 10);
    marker_pose_pub_ = nh_.advertise<visualization_msgs::Marker>("pose_markers", 10);
    
    kinect_pose_pub_ = nh_.advertise<geometry_msgs::PoseArray>("kinect_pose_array", 10);
    calibrated_pose_pub_ = nh_.advertise<geometry_msgs::PoseArray>("calibrated_pose_array", 10);
    
    
    pub_ = it_.advertise("image_out", 1);
    
    pattern_detector_.setPattern(cv::Size(6, 7), 0.027, CHESSBOARD);
    
    
    true_points_.push_back(Eigen::Vector3f(6*0.027,0,0));
    true_points_.push_back(Eigen::Vector3f(0,0,0));
    true_points_.push_back(Eigen::Vector3f(0,7*0.027,0));
    
    translation_old_.setZero();
    orientation_old_.setIdentity();
    cout << "Finished constructor." << endl;
  }

  void infoCb(const sensor_msgs::CameraInfoConstPtr& info_msg)
  {
    if (calibrated)
      return;
      
    //cam_info_ = info_msg;
    cam_model_.fromCameraInfo(info_msg);
    
    pattern_detector_.setCameraMatrices(cam_model_.intrinsicMatrix(), cam_model_.distortionCoeffs());
    calibrated = true;
    cout << "Got camera info!" << endl;
  }

  void imageCb(const sensor_msgs::ImageConstPtr& image_msg)
               //const sensor_msgs::CameraInfoConstPtr& info_msg)
  {
    if (!calibrated)
      return;
  
    ROS_INFO("Image callback!");
    try {
      bridge_ = cv_bridge::toCvCopy(image_msg, "mono8");
    }
    catch (cv_bridge::Exception& ex) {
      ROS_ERROR("[calibrate] Failed to convert image");
      return;
    }

    Eigen::Vector3f translation;
    Eigen::Quaternionf orientation;
    
    bool detected = pattern_detector_.detectPattern(bridge_->image, translation, orientation);
    
    if (detected)
    {
      //cout << "Translation: " << translation.transpose() << endl
       //    << "Rotation: " << orientation.toRotationMatrix() << endl;
     
      // If pattern detected, should probably do stuff.
      // Visualize the markers in 3D: 
           
      


      // Rotate from image coordinate frame to world coordinate frame
      Eigen::Matrix3f rotmat;
      rotmat << 0, 0, 1, -1, 0, 0, 0, -1, 0;
      Eigen::Quaternionf quat(rotmat);
      
      //translation = quat*translation;
      //orientation = quat*orientation;
      
      publishTargetMarker(translation, orientation);

    }
    
    tf::StampedTransform transform, kinect_transform;
    try {
      ros::Time acquisition_time = image_msg->header.stamp;
      ros::Duration timeout(1.0 / 30.0);
      
      // Get base_link transform
      tf_listener_.waitForTransform(world_frame, base_frame,
                                    acquisition_time, timeout);
      tf_listener_.lookupTransform(world_frame, base_frame,
                                   acquisition_time, transform);
                                   
      // Get the original kinect_link transform                             
      tf_listener_.waitForTransform(base_frame, camera_frame,
                                    acquisition_time, timeout);
      tf_listener_.lookupTransform(base_frame, camera_frame,
                                   acquisition_time, kinect_transform);
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
    
    Eigen::Vector3f kinect_translation(kinect_transform.getOrigin().x(), kinect_transform.getOrigin().y(), kinect_transform.getOrigin().z());
    Eigen::Quaternionf kinect_orientation(kinect_transform.getRotation().w(), kinect_transform.getRotation().x(), kinect_transform.getRotation().y(), kinect_transform.getRotation().z());
    
    
    //cout << endl << "Base Translation: " << base_translation.transpose() 
    //     << endl << "Base Quaternion: " << base_orientation.coeffs().transpose() << endl;
    
    pub_.publish(bridge_->toImageMsg());
    
    // Let's add this to our LM thing. And them optimize it. Yeaaaah!
    if (detected)
    {
      // Keyframing - could also do this by base position.
      // Whichever seems most convenient.
      //cout << endl << "Translation norm: " << (translation - translation_old_).norm()
      //      << " Quaternion norm: " << (orientation.angularDistance(orientation_old_)) << endl << endl;
      if ((translation - translation_old_).norm() > 0.01 || (orientation.angularDistance(orientation_old_)) > 0.01)
      {
        PointVector projected_points(true_points_.size());
        for (unsigned int i=0; i < true_points_.size(); i++)
        {
          projected_points[i] = orientation.toRotationMatrix()*true_points_[i] + translation;
        } 
        
        est_.addData(ObjectPose(base_translation, base_orientation), projected_points);
        
        Transform<float,3,Affine> guess(Translation3f(0,0,0));
        cout << endl << "Total cost with calibration: " << est_.computeTotalCost() 
              << " Without: "
             << est_.computeTotalCost(Transform<float,3,Affine>(kinect_orientation).pretranslate(kinect_translation)) << endl << endl;
              
        est_.computeTransform(guess);
        cout << endl << "Final transform with " << est_.base_pose.size() <<  " poses: " << endl << est_.getTransform().matrix() << endl;
        
        translation_old_ = translation;
        orientation_old_ = orientation;
        
        publishPoseMarkers(Transform<float,3,Affine>(kinect_orientation).pretranslate(kinect_translation), est_.getTransform());
        publishPoseLines(Transform<float,3,Affine>(kinect_orientation).pretranslate(kinect_translation), est_.getTransform());
      }
    }
  }
  
  void publishTargetMarker(Eigen::Vector3f& translation, Eigen::Quaternionf& orientation)
  {
    visualization_msgs::Marker target_marker;
    target_marker.header.frame_id = camera_frame;
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
    
    target_marker.pose.position.x = translation.x();
    target_marker.pose.position.y = translation.y();
    target_marker.pose.position.z = translation.z();
    
    target_marker.pose.orientation.x = orientation.x();
    target_marker.pose.orientation.y = orientation.y();
    target_marker.pose.orientation.z = orientation.z();
    target_marker.pose.orientation.w = orientation.w();
    
    marker_pub_.publish(target_marker);
  }
  
  void publishPoseLines(Eigen::Transform<float,3,Affine> kinect_transform, Eigen::Transform<float,3,Affine> calibrated_transform)
  {
    // Publish pose markers of all poses tracked in kinect frame and estimated kinect frame
    // See which ones are more self-consistent
    visualization_msgs::Marker kinect_poses, calibrated_poses;
    kinect_poses.header.frame_id = calibrated_poses.header.frame_id = world_frame;
    kinect_poses.header.stamp = calibrated_poses.header.stamp = ros::Time::now();
    kinect_poses.type = calibrated_poses.type = visualization_msgs::Marker::LINE_LIST;
    kinect_poses.id = 0;
    calibrated_poses.id = 1;
    
    kinect_poses.scale.x = 0.01;
    kinect_poses.color.r = 0;
    kinect_poses.color.g = 1;
    kinect_poses.color.b = 1;
    kinect_poses.color.a = 1;
    calibrated_poses.scale.x = 0.01;
    calibrated_poses.color.r = 1;
    calibrated_poses.color.g = 0;
    calibrated_poses.color.b = 1;
    calibrated_poses.color.a = 1;
    
    
    for (int i=0; i < est_.base_pose.size(); i++)
    {
      // Get the transforms and points for this iteration.
      Transform<float,3,Affine> kinect_pose_transform = (est_.base_pose[i].transform()*kinect_transform);
      Transform<float,3,Affine> calibrated_pose_transform = (est_.base_pose[i].transform()*calibrated_transform);
      kinect_pose_transform.translation().z() = 0;
      calibrated_pose_transform.translation().z() = 0;
      
      for (int j=0; j < est_.target_points[i].size(); j++)
      {
        geometry_msgs::Point pointk, pointc;
        Vector3f kinect_point = kinect_pose_transform*est_.target_points[i][j];
        Vector3f calibrated_point = calibrated_pose_transform*est_.target_points[i][j];
        
        pointk.x = kinect_point.x();
        pointk.y = kinect_point.y();
        pointk.z = kinect_point.z();
        
        pointc.x = calibrated_point.x();
        pointc.y = calibrated_point.y();
        pointc.z = calibrated_point.z();
        
        kinect_poses.points.push_back(pointk);
        calibrated_poses.points.push_back(pointc);
        
        if (j > 0 && j < est_.target_points[i].size()-1)
        {
          kinect_poses.points.push_back(pointk);
          calibrated_poses.points.push_back(pointc);
        }
      }
    }
    marker_pose_pub_.publish(kinect_poses);
    marker_pose_pub_.publish(calibrated_poses);
 
  }
  
  void publishPoseMarkers(Eigen::Transform<float,3,Affine> kinect_transform, Eigen::Transform<float,3,Affine> calibrated_transform)
  {
    // Publish pose markers of all poses tracked in kinect frame and estimated kinect frame
    // See which ones are more self-consistent
    geometry_msgs::PoseArray kinect_poses, calibrated_poses;
    kinect_poses.header.frame_id = calibrated_poses.header.frame_id = world_frame;
    kinect_poses.header.stamp = calibrated_poses.header.stamp = ros::Time::now();
    
    geometry_msgs::Pose kinect_pose;
    geometry_msgs::Pose calibrated_pose;
    
    //Eigen::Transformf kinect_transform;
    //Eigen::Transformf calibrated_transform;
    
    for (int i=0; i < est_.base_pose.size(); i++)
    {
      // Get the transforms and points for this iteration.
      Transform<float,3,Affine> kinect_pose_transform = (est_.base_pose[i].transform()*kinect_transform);
      Transform<float,3,Affine> calibrated_pose_transform = (est_.base_pose[i].transform()*calibrated_transform);
      
      Vector3f kinect_point = kinect_pose_transform*est_.target_points[i][0];
      Vector3f calibrated_point = calibrated_pose_transform*est_.target_points[i][0];
      
      Vector3f normal(0,0,-1), kinect_normal, calibrated_normal;
      Matrix3f normalMatrix = kinect_pose_transform.linear().inverse().transpose();
      kinect_normal = (normalMatrix * normal).normalized();
      normalMatrix = calibrated_pose_transform.linear().inverse().transpose();
      calibrated_normal = (normalMatrix * normal).normalized();
      
      Quaternionf kinect_quat = Quaternionf().setFromTwoVectors(normal, kinect_normal);
      Quaternionf calibrated_quat = Quaternionf().setFromTwoVectors(normal, calibrated_normal);
      
      Vector3f translation = kinect_pose_transform.translation();
      kinect_pose.position.x = kinect_point.x();
      kinect_pose.position.y = kinect_point.y();
      kinect_pose.position.z = 0;//kinect_point.z();
      
      Quaternionf orientation = Quaternionf(kinect_pose_transform.rotation());
      kinect_pose.orientation.x = orientation.x();
      kinect_pose.orientation.y = orientation.y();
      kinect_pose.orientation.z = orientation.z();
      kinect_pose.orientation.w = orientation.w();
      
      translation = calibrated_pose_transform.translation();
      calibrated_pose.position.x = calibrated_point.x();
      calibrated_pose.position.y = calibrated_point.y();
      calibrated_pose.position.z = 0;//calibrated_point.z();
      
      orientation = Quaternionf(calibrated_pose_transform.rotation());
      calibrated_pose.orientation.x = orientation.x();
      calibrated_pose.orientation.y = orientation.y();
      calibrated_pose.orientation.z = orientation.z();
      calibrated_pose.orientation.w = orientation.w();
    
      kinect_poses.poses.push_back(kinect_pose);
      calibrated_poses.poses.push_back(calibrated_pose);
    }
    
    kinect_pose_pub_.publish(kinect_poses);
    calibrated_pose_pub_.publish(calibrated_poses);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "calibrate_kinect_base");
  
  CalibrateExtrinsics cal;
  ros::spin();
}


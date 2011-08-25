/*********************************************************************          
 * Software License Agreement (BSD License)                                     
 *                                                                              
 *  Copyright (c) 2008, Willow Garage, Inc.                                     
 *  All rights reserved.                                                        
 *                                                                              
 *  Redistribution and use in source and binary forms, with or without          
 *  modification, are permitted provided that the following conditions          
 *  are met:                                                                    
 *                                                                              
 *   * Redistributions of source code must retain the above copyright           
 *     notice, this list of conditions and the following disclaimer.            
 *   * Redistributions in binary form must reproduce the above                  
 *     copyright notice, this list of conditions and the following              
 *     disclaimer in the documentation and/or other materials provided          
 *     with the distribution.                                                   
 *   * Neither the name of Willow Garage nor the names of its                   
 *     contributors may be used to endorse or promote products derived          
 *     from this software without specific prior written permission.            
 *                                                                              
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS         
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT           
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS           
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE              
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,         
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,        
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;            
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER            
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT          
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN           
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE             
 *  POSSIBILITY OF SUCH DAMAGE.                                                 
 **********************************************************************/

/* Author: Melonee Wise */

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include "turtlebot_actions/FindFiducialAction.h"
#include "turtlebot_actions/detect_calibration_pattern.h"

class FindFiducialAction
{
public:

  FindFiducialAction(std::string name) :
    as_(nh_, name),
    action_name_(name),
    it_(nh_)
  {
    //register the goal and feeback callbacks                                   
    as_.registerGoalCallback(boost::bind(&FindFiducialAction::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&FindFiducialAction::preemptCB, this));

  }

  ~FindFiducialAction(void)
  {
  }

  void goalCB()
  {
    ROS_INFO("%s: Received new goal", action_name_.c_str());

    typedef boost::shared_ptr<const turtlebot_actions::FindFiducialGoal> GoalPtr;
    GoalPtr goal = as_.acceptNewGoal();

    cv::Size grid_size(goal->pattern_width,goal->pattern_height);
    detector_.setPattern(grid_size, goal->pattern_size, Pattern(goal->pattern_type));

    ros::Duration(1.0).sleep();
    //subscribe to the image topic of interest
    std::string image_topic = goal->camera_name + "/image_rect";
    sub_ = it_.subscribeCamera(image_topic, 1, &FindFiducialAction::detectCB, this);

    pub_timer_ = nh_.createTimer(tf_listener_.getCacheLength() - ros::Duration(1.0), boost::bind(&FindFiducialAction::timeoutCB, this, _1),true);
  }

  void timeoutCB(const ros::TimerEvent& e)
  {
    if(sub_.getNumPublishers() == 0)
      ROS_INFO("%s: Aborted, there are no publishers on goal topic.", action_name_.c_str());    
    else
      ROS_INFO("%s: Aborted, there are publishers on goal topic, but detection took too long.", action_name_.c_str());    
    
    sub_.shutdown();
    as_.setAborted();
  }

  void preemptCB()
  {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted                                        
    pub_timer_.stop();
    as_.setPreempted();
    sub_.shutdown();
  }

  void detectCB(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg)
  {
    // make sure that the action is active                            
    if (!as_.isActive())
      return;

    ROS_INFO("%s: Received image, performing detection", action_name_.c_str());
    // Convert image message

    try
    {
      img_bridge_ = cv_bridge::toCvCopy(image_msg, "mono8");
    }
    catch (cv_bridge::Exception& ex)
    {
      ROS_ERROR("[calibrate] Failed to convert image");
      return;
    }


    ROS_INFO("%s: created cv::Mat", action_name_.c_str());
    cam_model_.fromCameraInfo(info_msg);
    detector_.setCameraMatrices(cam_model_.intrinsicMatrix(), cam_model_.distortionCoeffs());

    Eigen::Vector3f translation;
    Eigen::Quaternionf orientation;

    if (detector_.detectPattern(img_bridge_->image, translation, orientation))
    {
      ROS_INFO("detected fiducial");
      tf::Transform fiducial_transform;
      fiducial_transform.setOrigin( tf::Vector3(translation.x(), translation.y(), translation.z()) );
      fiducial_transform.setRotation( tf::Quaternion(orientation.x(), orientation.y(), orientation.z(), orientation.w()) );

      tf::StampedTransform fiducial_transform_stamped(fiducial_transform, image_msg->header.stamp, cam_model_.tfFrame(), "fiducial_frame");
      tf_broadcaster_.sendTransform(fiducial_transform_stamped);

      tf::Pose fiducial_pose;
      fiducial_pose.setRotation(tf::Quaternion(0, 0, 0, 1.0));
      fiducial_pose = fiducial_transform * fiducial_pose;
      tf::Stamped<tf::Pose> fiducial_pose_stamped(fiducial_pose, image_msg->header.stamp, cam_model_.tfFrame());

      tf::poseStampedTFToMsg(fiducial_pose_stamped, result_.pose);
      as_.setSucceeded(result_);
      pub_timer_.stop();
      sub_.shutdown();
    }

  }

protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<turtlebot_actions::FindFiducialAction> as_;
  std::string action_name_;
  PatternDetector detector_;

  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber sub_;
  cv_bridge::CvImagePtr img_bridge_;
  image_geometry::PinholeCameraModel cam_model_;

  tf::TransformListener tf_listener_;
  tf::TransformBroadcaster tf_broadcaster_;

  ros::Timer pub_timer_;

  // create messages that are used to published feedback/result                 
  turtlebot_actions::FindFiducialFeedback feedback_;
  turtlebot_actions::FindFiducialResult result_;


};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "find_fiducial_pose");

  FindFiducialAction find_fiducial(ros::this_node::getName());
  ros::spin();

  return 0;
}




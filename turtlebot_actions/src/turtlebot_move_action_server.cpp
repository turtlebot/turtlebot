/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <actionlib/server/simple_action_server.h>
#include <turtlebot_actions/TurtlebotMoveAction.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <cmath>

class MoveActionServer
{
private:
    
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<turtlebot_actions::TurtlebotMoveAction> as_;
  std::string action_name_;

  turtlebot_actions::TurtlebotMoveFeedback     feedback_;
  turtlebot_actions::TurtlebotMoveResult       result_;
  turtlebot_actions::TurtlebotMoveGoalConstPtr goal_;
  
  ros::Subscriber       sub_;
  ros::Publisher        cmd_vel_pub_;
  tf::TransformListener listener_;
  
  // Parameters
  std::string base_frame;
  std::string odom_frame;
  double turn_rate;
  double forward_rate;
  
public:
  MoveActionServer(const std::string name) : 
    nh_("~"), as_(nh_, name, false), action_name_(name)
  {
    // Get parameters
    nh_.param<std::string>("base_frame", base_frame, "base_link");
    nh_.param<std::string>("odom_frame", odom_frame, "odom");
    nh_.param<double>("turn_rate", turn_rate, 0.75);
    nh_.param<double>("forward_rate", forward_rate, 0.25);
    
    //register the goal and feeback callbacks
    as_.registerGoalCallback(boost::bind(&MoveActionServer::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&MoveActionServer::preemptCB, this));
    
    as_.start();
    
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  }

  void goalCB()
  {
    // accept the new goal
    feedback_.forward_distance = 0.0;
    feedback_.turn_distance = 0.0;
    
    result_.forward_distance = 0.0;
    result_.turn_distance = 0.0;
    
    goal_ = as_.acceptNewGoal();
    
    if (!turnOdom(goal_->turn_distance))
    { 
      as_.setAborted(result_);
      return;
    }
    
    if (driveForwardOdom(goal_->forward_distance))
      as_.setSucceeded(result_);
    else
      as_.setAborted(result_);
  }

  void preemptCB()
  {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    as_.setPreempted();
  }

  bool driveForwardOdom(double distance)
  {
    // If the distance to travel is negligble, don't even try.
    if (fabs(distance) < 0.01)
      return true;
    
    //we will record transforms here
    tf::StampedTransform start_transform;
    tf::StampedTransform current_transform;
  
    try
    {
      //wait for the listener to get the first message
      listener_.waitForTransform(base_frame, odom_frame, 
                                 ros::Time::now(), ros::Duration(1.0));
      
      //record the starting transform from the odometry to the base frame
      listener_.lookupTransform(base_frame, odom_frame, 
                                ros::Time(0), start_transform);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
      return false;
    }
    
    //we will be sending commands of type "twist"
    geometry_msgs::Twist base_cmd;
    //the command will be to go forward at 0.25 m/s
    base_cmd.linear.y = base_cmd.angular.z = 0;
    base_cmd.linear.x = forward_rate;
    
    if (distance < 0)
      base_cmd.linear.x = -base_cmd.linear.x;
    
    ros::Rate rate(25.0);
    bool done = false;
    while (!done && nh_.ok() && as_.isActive())
    {
      //send the drive command
      cmd_vel_pub_.publish(base_cmd);
      rate.sleep(); 
      //get the current transform
      try
      {
        listener_.lookupTransform(base_frame, odom_frame, 
                                  ros::Time(0), current_transform);
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s",ex.what());
        break;
      }
      //see how far we've traveled
      tf::Transform relative_transform = 
        start_transform.inverse() * current_transform;
      double dist_moved = relative_transform.getOrigin().length();
      
      // Update feedback and result.
      feedback_.forward_distance = dist_moved;
      result_.forward_distance = dist_moved;
      as_.publishFeedback(feedback_);

      if(fabs(dist_moved) > fabs(distance))
      {
        done = true;
      }
    }
    if (done) return true;
    return false;
  }

  bool turnOdom(double radians)
  {
    // If the distance to travel is negligble, don't even try.
    if (fabs(radians) < 0.01)
      return true;
  
    while(radians < -M_PI) radians += 2*M_PI;
    while(radians > M_PI) radians -= 2*M_PI;

    //we will record transforms here
    tf::StampedTransform start_transform;
    tf::StampedTransform current_transform;

    try
    {
      //wait for the listener to get the first message
      listener_.waitForTransform(base_frame, odom_frame, 
                                 ros::Time::now(), ros::Duration(1.0));

      //record the starting transform from the odometry to the base frame
      listener_.lookupTransform(base_frame, odom_frame, 
                                ros::Time(0), start_transform);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
      return false;
    }
    
    //we will be sending commands of type "twist"
    geometry_msgs::Twist base_cmd;
    //the command will be to turn at 0.75 rad/s
    base_cmd.linear.x = base_cmd.linear.y = 0.0;
    base_cmd.angular.z = turn_rate;
    if (radians < 0)
      base_cmd.angular.z = -turn_rate;
    
    //the axis we want to be rotating by
    tf::Vector3 desired_turn_axis(0,0,1);
    
    ros::Rate rate(25.0);
    bool done = false;
    while (!done && nh_.ok() && as_.isActive())
    {
      //send the drive command
      cmd_vel_pub_.publish(base_cmd);
      rate.sleep();
      //get the current transform
      try
      {
        listener_.lookupTransform(base_frame, odom_frame, 
                                  ros::Time(0), current_transform);
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s",ex.what());
        break;
      }
      tf::Transform relative_transform = 
        start_transform.inverse() * current_transform;
      tf::Vector3 actual_turn_axis = 
        relative_transform.getRotation().getAxis();
      double angle_turned = relative_transform.getRotation().getAngle();
      
      // Update feedback and result.
      feedback_.turn_distance = angle_turned;
      result_.turn_distance = angle_turned;
      as_.publishFeedback(feedback_);
      
      if ( fabs(angle_turned) < 1.0e-2) continue;

      //if ( actual_turn_axis.dot( desired_turn_axis ) < 0 ) 
      //  angle_turned = 2 * M_PI - angle_turned;

      if (fabs(angle_turned) > fabs(radians)) done = true;
    }
    if (done) return true;
    return false;
  }


};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "turtlebot_move_action_server");

  MoveActionServer server("turtlebot_move");
  ros::spin();

  return 0;
}


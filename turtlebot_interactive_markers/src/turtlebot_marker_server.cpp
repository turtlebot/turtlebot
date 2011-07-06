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
#include <interactive_markers/interactive_marker_server.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>

using namespace visualization_msgs;

// To do: split into header file
class TurtlebotMarkerServer
{
  public:
    TurtlebotMarkerServer()
      : server("turtle_marker_server")
    {
      ROS_INFO_STREAM("Started constructor.");
      vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
      ROS_INFO_STREAM("Started interactive markers.");
      createInteractiveMarkers();
      ROS_INFO_STREAM("Finished constructor.");
    }
    
    void processFeedback(
        const InteractiveMarkerFeedbackConstPtr &feedback );
  
  
  private:
    void createInteractiveMarkers();
  
    
    ros::NodeHandle nh;
    ros::Publisher vel_pub;
    interactive_markers::InteractiveMarkerServer server;
};

void TurtlebotMarkerServer::processFeedback(
    const InteractiveMarkerFeedbackConstPtr &feedback )
{
  // To do: change this to send a move command to the turtlebot
  ROS_INFO_STREAM( feedback->marker_name << " is now at "
      << feedback->pose.position.x << ", " << feedback->pose.position.y 
      << " orientation: " << feedback->pose.orientation.x << ", " 
      << feedback->pose.orientation.y << ", "
      << feedback->pose.orientation.z << ", "
      << feedback->pose.orientation.w << ", " );
      
  
  // To do: handle angular change (figure out how quaternion changed?)
  // To do: make the marker snap back to turtlebot
  server.setPose("turtlebot_marker", geometry_msgs::Pose());
  
  server.applyChanges();
  
  
}

void TurtlebotMarkerServer::createInteractiveMarkers()
{ 
  // create an interactive marker for our server
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "/base_link";
  int_marker.name = "turtlebot_marker";
  int_marker.description = "Move the turtlebot";

  // create a grey box marker
  Marker box_marker;
  box_marker.type = Marker::CUBE;
  box_marker.scale.x = 0.2;
  box_marker.scale.y = 0.2;
  box_marker.scale.z = 0.2;
  box_marker.color.r = 1.0;
  box_marker.color.g = 0.3;
  box_marker.color.b = 0.3;
  box_marker.color.a = 1.0;

  // create a non-interactive control which contains the box
  InteractiveMarkerControl box_control;
  box_control.always_visible = true;
  box_control.markers.push_back( box_marker );

  // add the control to the interactive marker 
  int_marker.controls.push_back( box_control );
  
  InteractiveMarkerControl control;

  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.name = "move_x";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.name = "rotate_z";
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.name = "move_y";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);
  
  // add the interactive marker to our collection &
  // tell the server to call processFeedback() when feedback arrives for it
  server.insert(int_marker, boost::bind( &TurtlebotMarkerServer::processFeedback, this, _1 ));
  
  // 'commit' changes and send to all clients
  server.applyChanges();
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "turtlebot_marker_server");

  ROS_INFO_STREAM("Init.");
  TurtlebotMarkerServer turtleserver;
  
  ROS_INFO_STREAM("Starting spin.");

  // start the ROS main loop
  ros::spin();
}

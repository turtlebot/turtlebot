/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2011-2012, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  FOOTPRINTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include <math.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>

class LaserFootprintFilter
{
public:
  LaserFootprintFilter()
    : nh_("~"), listener_(ros::Duration(10))
  {
    scan_filtered_pub_ = nh_.advertise<sensor_msgs::LaserScan>("/scan_filtered", 1);
    scan_sub_ = nh_.subscribe("/scan", 1000, &LaserFootprintFilter::update, this);

    nh_.param<double>("footprint_inscribed_radius", inscribed_radius_, 0.16495*1.1);
    nh_.param<std::string>("base_frame", base_frame_, "/base_link");
  }

  void update(const sensor_msgs::LaserScan& input_scan)
  {
    sensor_msgs::LaserScan filtered_scan;
    filtered_scan = input_scan;

    double angle = filtered_scan.angle_min - filtered_scan.angle_increment;
    geometry_msgs::PointStamped p_input;
    p_input.header = input_scan.header;

    for(size_t i=0; i < filtered_scan.ranges.size(); i++)
    {
        angle += filtered_scan.angle_increment;
        if(filtered_scan.ranges[i] >= filtered_scan.range_max) continue;

        p_input.point.x = cos(angle) * filtered_scan.ranges[i];
        p_input.point.y = sin(angle) * filtered_scan.ranges[i];

        geometry_msgs::PointStamped p_transformed;        
        try{
            listener_.transformPoint(base_frame_, p_input, p_transformed);
        }catch(tf::TransformException &ex){
            ROS_ERROR("Received an exception trying to transform a point: %s", ex.what());
            return;
        }
    
        if( inFootprint(p_transformed) )
        {
            filtered_scan.ranges[i] = filtered_scan.range_max + 1.0;
        }

    }

    scan_filtered_pub_.publish(filtered_scan);
  }

  // Filter out circular area
  bool inFootprint(const geometry_msgs::PointStamped& scan_pt)
  {
    // Do a radius instead of a box.
    if (sqrt(scan_pt.point.x*scan_pt.point.x + scan_pt.point.y*scan_pt.point.y) > inscribed_radius_)
      return false;
    return true;
  }

private:
  ros::NodeHandle nh_;
  tf::TransformListener listener_;
  double inscribed_radius_;
  std::string base_frame_;
  ros::Publisher scan_filtered_pub_;
  ros::Publisher debug_pub_;
  ros::Subscriber scan_sub_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "laser_footprint_filter");

  LaserFootprintFilter filter;
  ros::spin();
}


/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2011, Willow Garage, Inc.
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

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud.h>
#include <laser_geometry/laser_geometry.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/ros/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/impl/transforms.hpp>

struct PointXYZIndex
{
  PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
  int32_t index;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIndex,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (int32_t, index, index)
)

typedef pcl::PointCloud<PointXYZIndex> PointCloud;

class LaserFootprintFilter
{
public:
  LaserFootprintFilter()
    : nh_("~"), tf_(ros::Duration(10))
  {
    scan_filtered_pub_ = nh_.advertise<sensor_msgs::LaserScan>("/scan_filtered", 1);
    scan_sub_ = nh_.subscribe("/scan", 1000, &LaserFootprintFilter::update, this);

    nh_.param<double>("footprint_inscribed_radius", inscribed_radius_, 0.16495);
    nh_.param<std::string>("base_frame", base_frame_, "/base_link");
  }

  void update(const sensor_msgs::LaserScan& input_scan)
  {
    sensor_msgs::LaserScan filtered_scan;
    filtered_scan = input_scan;
    sensor_msgs::PointCloud2 cloud_message;
    PointCloud laser_cloud_in, laser_cloud;

    // Get the pointcloud from the laser scan
    projector_.projectLaser(input_scan, cloud_message);
    
    pcl::fromROSMsg(cloud_message, laser_cloud_in);

    // Transform the pointcloud
    pcl_ros::transformPointCloud (base_frame_, input_scan.header.stamp, laser_cloud_in, base_frame_, laser_cloud, tf_);

    // Go through each point in the pointcloud and keep or remove corresponding
    // points in the original scan as necessary.
    for (unsigned int i=0; i < laser_cloud.points.size(); i++)  
    {
      // Check if the point is in the footprint
      if (inFootprint(laser_cloud.points[i]))
      {
        int index = laser_cloud.points[i].index;
        filtered_scan.ranges[index] = filtered_scan.range_max + 1.0; // If so, then make it a value bigger than the max range
      }
    }
    
    scan_filtered_pub_.publish(filtered_scan);
  }

  // Filter out circular area
  bool inFootprint(const PointXYZIndex& scan_pt)
  {
    // Do a radius instead of a box.
    if (sqrt(scan_pt.x*scan_pt.x + scan_pt.y*scan_pt.y) > inscribed_radius_)
      return false;
    return true;
  }

private:
  ros::NodeHandle nh_;
  tf::TransformListener tf_;
  laser_geometry::LaserProjection projector_;
  double inscribed_radius_;
  std::string base_frame_;
  ros::Publisher scan_filtered_pub_;
  ros::Publisher debug_pub_;
  ros::Subscriber scan_sub_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "filter_laser_footprint");

  LaserFootprintFilter filter;
  ros::spin();
}


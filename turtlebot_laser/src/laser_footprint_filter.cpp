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

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIndex,           // here we assume a XYZ + "test" (as fields)
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
    : tf_(ros::Duration(10000000)), first_(true)
  {
    scan_filtered_pub_ = nh_.advertise<sensor_msgs::LaserScan>("/scan_filtered", 1);
    scan_sub_ = nh_.subscribe("/scan", 1000, &LaserFootprintFilter::update, this);
    debug_pub_ = nh_.advertise<PointCloud> ("debug_pointcloud", 1);

    
    if (!nh_.getParam("footprint_inscribed_radius", inscribed_radius_))
      inscribed_radius_ = 0.16495;
  }

  virtual ~LaserFootprintFilter()
  { 
  }

  void update(const sensor_msgs::LaserScan& input_scan)
  {
    sensor_msgs::LaserScan filtered_scan;
    filtered_scan = input_scan;
    sensor_msgs::PointCloud2 cloud_message;
    PointCloud laser_cloud_in, laser_cloud;

    // Get the pointcloud from the laser scan
    projector_.projectLaser(input_scan, cloud_message);
    
    debug_pub_.publish(cloud_message);
    pcl::fromROSMsg(cloud_message, laser_cloud_in);
    
    /* if (first_)
    {
      // Get the transform from tf_listener
      // To do: make this just execute the first time.
      try 
      {
         tf_.waitForTransform ("/base_link", input_scan.header.frame_id, input_scan.header.stamp, ros::Duration(10));
      }
      catch(tf::TransformException& ex){
        ROS_ERROR("Transform unavailable %s", ex.what());
      }
      ROS_INFO("Acquired laser transform.");
      first_ = false;
      
    } */

    // Not 100% sure what fixed_frame should be.
    pcl_ros::transformPointCloud ("/base_link", input_scan.header.stamp, laser_cloud_in, "/base_link", laser_cloud, tf_);

    // Transform the pointcloud
    //transformPointCloud (const pcl::PointCloud< PointT > &cloud_in, pcl::PointCloud< PointT > &cloud_out, const tf::Transform &transform);

    for (unsigned int i=0; i < laser_cloud.points.size(); i++)  
    {
      if (inFootprint(laser_cloud.points[i]))
      {
        int index = laser_cloud.points[i].index;
        filtered_scan.ranges[index] = filtered_scan.range_max + 1.0; // If so, then make it a value bigger than the max range
      }
    }
    
    scan_filtered_pub_.publish(filtered_scan);
  }


  // To do: Currently removes everything in a box: since our robot is circular,
  // change this to be a circle instead.
  bool inFootprint(const PointXYZIndex& scan_pt)
  {
    // Do a radius instead of a box.
    if (pow(scan_pt.x*scan_pt.x + scan_pt.y*scan_pt.y,2) > inscribed_radius_)
      return false;
    return true;
  }

private:
  tf::TransformListener tf_;
  laser_geometry::LaserProjection projector_;
  double inscribed_radius_;
  ros::Publisher scan_filtered_pub_;
  ros::Publisher debug_pub_;
  ros::Subscriber scan_sub_;
  ros::NodeHandle nh_;
  bool first_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "filter_laser_footprint");

  LaserFootprintFilter filter;

  // start the ROS main loop
  ros::spin();
}


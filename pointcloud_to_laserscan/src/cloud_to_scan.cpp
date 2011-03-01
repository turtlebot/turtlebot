/*                                                                                                                       
 * Copyright (c) 2010, Willow Garage, Inc.                                                                               
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

#include "ros/ros.h"
#include "pluginlib/class_list_macros.h"
#include "nodelet/nodelet.h"
#include "sensor_msgs/LaserScan.h"
#include "pcl/point_cloud.h"
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/ros/conversions.h"




namespace pointcloud_to_laserscan
{
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class CloudToScan : public nodelet::Nodelet
{
public:
  //Constructor
  CloudToScan(): min_height_(0.10), max_height_(0.15), u_min_(100), u_max_(150), output_frame_id_("/openi_depth_frame")
  {
  };

private:
  virtual void onInit()
  {
    ros::NodeHandle& nh = getNodeHandle();
    ros::NodeHandle& private_nh = getPrivateNodeHandle();
    
    private_nh.getParam("min_height", min_height_);
    private_nh.getParam("max_height", max_height_);

    private_nh.getParam("row_min", u_min_);
    private_nh.getParam("row_max", u_max_);

    private_nh.getParam("output_frame_id", output_frame_id_);
    pub_ = nh.advertise<sensor_msgs::LaserScan>("scan", 10);
    sub_ = nh.subscribe<PointCloud>("cloud", 10, &CloudToScan::callback, this);
  };

  void callback(const PointCloud::ConstPtr& cloud)
  {
    sensor_msgs::LaserScanPtr output(new sensor_msgs::LaserScan());
    NODELET_DEBUG("Got cloud");
    //Copy Header
    output->header = cloud->header;
    output->header.frame_id = output_frame_id_;
    output->angle_min = -M_PI/2;
    output->angle_max = M_PI/2;
    output->angle_increment = M_PI/180.0/2.0;
    output->time_increment = 0.0;
    output->scan_time = 1.0/30.0;
    output->range_min = 0.45;
    output->range_max = 10.0;
    
    uint32_t ranges_size = std::ceil((output->angle_max - output->angle_min) / output->angle_increment);
    output->ranges.assign(ranges_size, output->range_max + 1.0);

    //pcl::PointCloud<pcl::PointXYZ> cloud;
    //pcl::fromROSMsg(*input, cloud);
    
    for (PointCloud::const_iterator it = cloud->begin(); it != cloud->end(); ++it)
    {
      const float &x = it->x;
      const float &y = it->y;
      const float &z = it->z;
    
    /*    for (uint32_t u = std::max((uint32_t)u_min_, 0U); u < std::min((uint32_t)u_max_, cloud.height - 1); u++)
      for (uint32_t v = 0; v < cloud.width -1; v++)
      {
        //NODELET_ERROR("%d %d,  %d %d", u, v, cloud.height, cloud.width);
        pcl::PointXYZ point = cloud.at(v,u); ///WTF Column major?
        const float &x = point.x;
        const float &y = point.y;
        const float &z = point.z;
    */  

      if ( std::isnan(x) || std::isnan(y) || std::isnan(z) )
      {
        NODELET_DEBUG("rejected for nan in point(%f, %f, %f)\n", x, y, z);
        continue;  
      }
      if (-y > max_height_ || -y < min_height_)
      {
        NODELET_DEBUG("rejected for height %f not in range (%f, %f)\n", x, min_height_, max_height_);
        continue;
      }
      double angle = -atan2(x, z);
      if (angle < output->angle_min || angle > output->angle_max)
      {
        NODELET_DEBUG("rejected for angle %f not in range (%f, %f)\n", angle, output->angle_min, output->angle_max);
        continue;
      }
      int index = (angle - output->angle_min) / output->angle_increment;
      //printf ("index xyz( %f %f %f) angle %f index %d\n", x, y, z, angle, index);
      double range_sq = z*z+x*x;
      if (output->ranges[index] * output->ranges[index] > range_sq)
        output->ranges[index] = sqrt(range_sq);
      
      
      }
    
    pub_.publish(output);
  }


  double min_height_, max_height_;
  int32_t u_min_, u_max_;
  std::string output_frame_id_;

  ros::Publisher pub_;
  ros::Subscriber sub_;
  
};


PLUGINLIB_DECLARE_CLASS(pointcloud_to_laserscan, CloudToScan, pointcloud_to_laserscan::CloudToScan, nodelet::Nodelet);
}

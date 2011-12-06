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
#include "dynamic_reconfigure/server.h"
#include "pointcloud_to_laserscan/CloudScanConfig.h"

namespace pointcloud_to_laserscan
{
class CloudToScan : public nodelet::Nodelet
{
public:
  //Constructor
  CloudToScan(): min_height_(0.10),
                 max_height_(0.15),
                 angle_min_(-M_PI/2),
                 angle_max_(M_PI/2),
                 angle_increment_(M_PI/180.0/2.0),
                 scan_time_(1.0/30.0),
                 range_min_(0.45),
                 range_max_(10.0),
                 output_frame_id_("/kinect_depth_frame")
  {
  };

  ~CloudToScan()
  {
    delete srv_;
  }

private:
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

  boost::mutex connect_mutex_;
  // Dynamic reconfigure server
  dynamic_reconfigure::Server<pointcloud_to_laserscan::CloudScanConfig>* srv_;

  virtual void onInit()
  {
    nh_ = getNodeHandle();
    ros::NodeHandle& private_nh = getPrivateNodeHandle();

    private_nh.getParam("min_height", min_height_);
    private_nh.getParam("max_height", max_height_);

    private_nh.getParam("angle_min", angle_min_);
    private_nh.getParam("angle_max", angle_max_);
    private_nh.getParam("angle_increment", angle_increment_);
    private_nh.getParam("scan_time", scan_time_);
    private_nh.getParam("range_min", range_min_);
    private_nh.getParam("range_max", range_max_);

    range_min_sq_ = range_min_ * range_min_;

    private_nh.getParam("output_frame_id", output_frame_id_);

    srv_ = new dynamic_reconfigure::Server<pointcloud_to_laserscan::CloudScanConfig>(private_nh);
    dynamic_reconfigure::Server<pointcloud_to_laserscan::CloudScanConfig>::CallbackType f = boost::bind(&CloudToScan::reconfigure, this, _1, _2);
    srv_->setCallback(f);

    // Lazy subscription to point cloud topic
    ros::AdvertiseOptions scan_ao = ros::AdvertiseOptions::create<sensor_msgs::LaserScan>(
      "scan", 10,
      boost::bind( &CloudToScan::connectCB, this),
      boost::bind( &CloudToScan::disconnectCB, this), ros::VoidPtr(), nh_.getCallbackQueue());

    boost::lock_guard<boost::mutex> lock(connect_mutex_);
    pub_ = nh_.advertise(scan_ao);
  };

  void connectCB() {
      boost::lock_guard<boost::mutex> lock(connect_mutex_);
      if (pub_.getNumSubscribers() > 0) {
          NODELET_DEBUG("Connecting to point cloud topic.");
          sub_ = nh_.subscribe<PointCloud>("cloud", 10, &CloudToScan::callback, this);
      }
  }

  void disconnectCB() {
      boost::lock_guard<boost::mutex> lock(connect_mutex_);
      if (pub_.getNumSubscribers() == 0) {
          NODELET_DEBUG("Unsubscribing from point cloud topic.");
          sub_.shutdown();
      }
  }

  void reconfigure(pointcloud_to_laserscan::CloudScanConfig &config, uint32_t level)
  {
    min_height_ = config.min_height;
    max_height_ = config.max_height;
    angle_min_ = config.angle_min;
    angle_max_ = config.angle_max;
    angle_increment_ = config.angle_increment;
    scan_time_ = config.scan_time;
    range_min_ = config.range_min;
    range_max_ = config.range_max;

    range_min_sq_ = range_min_ * range_min_;
  }

  void callback(const PointCloud::ConstPtr& cloud)
  {
    sensor_msgs::LaserScanPtr output(new sensor_msgs::LaserScan());
    output->header = cloud->header;
    output->header.frame_id = output_frame_id_; // Set output frame. Point clouds come from "optical" frame, scans come from corresponding mount frame
    output->angle_min = angle_min_;
    output->angle_max = angle_max_;
    output->angle_increment = angle_increment_;
    output->time_increment = 0.0;
    output->scan_time = scan_time_;
    output->range_min = range_min_;
    output->range_max = range_max_;

    uint32_t ranges_size = std::ceil((output->angle_max - output->angle_min) / output->angle_increment);
    output->ranges.assign(ranges_size, output->range_max + 1.0);

    for (PointCloud::const_iterator it = cloud->begin(); it != cloud->end(); ++it)
    {
      const float &x = it->x;
      const float &y = it->y;
      const float &z = it->z;

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

      double range_sq = z*z+x*x;
      if (range_sq < range_min_sq_) {
        NODELET_DEBUG("rejected for range %f below minimum value %f. Point: (%f, %f, %f)", range_sq, range_min_sq_, x, y, z);
        continue;
      }

      double angle = -atan2(x, z);
      if (angle < output->angle_min || angle > output->angle_max)
      {
        NODELET_DEBUG("rejected for angle %f not in range (%f, %f)\n", angle, output->angle_min, output->angle_max);
        continue;
      }
      int index = (angle - output->angle_min) / output->angle_increment;


      if (output->ranges[index] * output->ranges[index] > range_sq)
        output->ranges[index] = sqrt(range_sq);
      }

    pub_.publish(output);
  }


  double min_height_, max_height_, angle_min_, angle_max_, angle_increment_, scan_time_, range_min_, range_max_, range_min_sq_;
  std::string output_frame_id_;

  ros::NodeHandle nh_;
  ros::Publisher pub_;
  ros::Subscriber sub_;

};

PLUGINLIB_DECLARE_CLASS(pointcloud_to_laserscan, CloudToScan, pointcloud_to_laserscan::CloudToScan, nodelet::Nodelet);
}

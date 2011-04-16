#include "ros/ros.h"
#include "pluginlib/class_list_macros.h" 
#include "nodelet/nodelet.h" 
#include <geometry_msgs/Twist.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>


namespace turtlebot_follower {
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

  class TurtlebotFollower : public nodelet::Nodelet 
  {
  public:
    TurtlebotFollower() : min_y_(0.1), max_y_(0.5),
			  min_x_(-0.2), max_x_(0.2),
			  max_z_(0.8), goal_z_(0.6),
			  z_scale_(1.0), x_scale_(5.0) {

    }

  private:
    double min_y_, max_y_, min_x_, max_x_;
    double max_z_, goal_z_, z_scale_, x_scale_;

    virtual void onInit() {
      ros::NodeHandle& nh = getNodeHandle();
      ros::NodeHandle& private_nh = getPrivateNodeHandle();

      private_nh.getParam("min_y", min_y_);
      private_nh.getParam("max_y", max_y_);
      private_nh.getParam("min_x", min_x_);
      private_nh.getParam("max_z", max_z_);
      private_nh.getParam("goal_z", goal_z_);
      private_nh.getParam("z_scale", z_scale_);
      private_nh.getParam("x_scale", x_scale_);
      
      cmdpub_ = nh.advertise<geometry_msgs::Twist> ("turtlebot_node/cmd_vel", 1);
      sub_= nh.subscribe<PointCloud>("/camera/depth/points", 1, &TurtlebotFollower::cloudcb, this);

    }

    void cloudcb(const PointCloud::ConstPtr&  cloud){
      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
      unsigned int n = 0;
      BOOST_FOREACH (const pcl::PointXYZ& pt, cloud->points) {

	if ( !std::isnan(x) && !std::isnan(y) && !std::isnan(z) )  {
	  //printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
	  if (-pt.y > min_y_ && -pt.y < max_y_) {
	    if (pt.x < max_x_ && pt.x > min_x_) {
	      if (pt.z < max_z_) {
		x += pt.x;
		y += pt.y;
		z += pt.z;
		n++;
	      }
	    }
	  }
	}
      }
      if (n) { 
	x /= n; y /= n; z /= n;  

	//printf("%f %f %f %d\n", x, y, z, n);

	geometry_msgs::Twist cmd;
	
	if (n) {
	  cmd.linear.x = (z - goal_z_) * z_scale_;
	  cmd.angular.z = -x * x_scale_;
	  cmdpub_.publish(cmd);
	  
	}
      } else {
	ROS_INFO("No object");
	cmdpub_.publish(geometry_msgs::Twist());
      }


    }


    ros::Subscriber sub_;
    ros::Publisher cmdpub_;
  };

  PLUGINLIB_DECLARE_CLASS(turtlebot_follower, TurtlebotFollower, turtlebot_follower::TurtlebotFollower, nodelet::Nodelet); 

}

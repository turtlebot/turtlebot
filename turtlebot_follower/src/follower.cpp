#include "ros/ros.h"
#include "pluginlib/class_list_macros.h" 
#include "nodelet/nodelet.h" 
#include <geometry_msgs/Twist.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>


namespace turtlebot_follower {

  class TurtlebotFollower : public nodelet::Nodelet 
  {
  public:
    TurtlebotFollower() : min_y_(0.1), max_y_(0.5) {
    }

  private:
    double min_y_, max_y_;

    virtual void onInit() {
      ros::NodeHandle& nh = getNodeHandle();
      ros::NodeHandle& private_nh = getPrivateNodeHandle();
      
      cmdpub_ = nh.advertise<geometry_msgs::Twist> ("/cmd_vel", 1);
      sub_= nh.subscribe("/camera/rgb/points", 1, &TurtlebotFollower::cloudcb, this);

    }

    void cloudcb(const sensor_msgs::PointCloud2::ConstPtr  &scan){
      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
      unsigned int n = 0;
      pcl::PointCloud<pcl::PointXYZ> cloud;
      pcl::fromROSMsg(*scan,cloud);
      BOOST_FOREACH (const pcl::PointXYZ& pt, cloud.points) {

	if ( !std::isnan(x) && !std::isnan(y) && !std::isnan(z) )  {
	  //printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
	  if (-pt.y > min_y_ && -pt.y < max_y_) {
	    if (pt.x < 0.2 && pt.x > -0.2) {
	      if (pt.z < 0.8) {
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

	printf("%f %f %f %d\n", x, y, z, n);

	geometry_msgs::Twist cmd;
	
	if (n) {
	  cmd.linear.x = z - 0.6;
	  cmd.angular.z = -x * 5.0;
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

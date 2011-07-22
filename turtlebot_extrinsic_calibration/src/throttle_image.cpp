#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

class ImageThrottle
{
public:
  //Constructor
  ImageThrottle(): max_update_rate_(0)
  {
    ros::NodeHandle nh;
    
    nh.getParam("~max_rate", max_update_rate_);

    pub_ = nh.advertise<sensor_msgs::Image>("image_out", 10);
    sub_ = nh.subscribe("image_in", 10, &ImageThrottle::callback, this);
  };

private:
  ros::Time last_update_;
  double max_update_rate_;

  void callback(const sensor_msgs::ImageConstPtr& image)
  {
    if (max_update_rate_ > 0.0)
    {
      ROS_DEBUG("update set to %f", max_update_rate_);
      if ( last_update_ + ros::Duration(1.0/max_update_rate_) > ros::Time::now())
      {
        ROS_DEBUG("throttle last update at %f skipping", last_update_.toSec());
        return;
      }
    }
    else
      ROS_DEBUG("update_rate unset continuing");
    
    last_update_ = ros::Time::now();
    pub_.publish(image);
  }


  ros::Publisher pub_;
  ros::Subscriber sub_;
  
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "throttle_image");
  
  ImageThrottle throttle;
  ros::spin();
}

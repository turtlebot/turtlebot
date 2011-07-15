#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry> 

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>


using namespace Eigen;
using namespace std;


int main( int argc, char** argv )
{
  // define transform between baselink and kinect.
  Transform<float, 3, Affine> t_base_kinect(Translation<float,3>(-.1, 0, .025));
  
  // define transform between baselink pose 1 and baselink pose 2:
  Transform<float, 3, Affine> t_base_1_2(Translation<float,3>(-1, 0, 0));
  
  // define transform between kinect and object in pose 1:
  Transform<float, 3, Affine> t_kinect_obj_1(Translation<float,3>(2, 0, 0));
  
  // calculate transform between kinect 1 and kinect 2:
  Transform<float, 3, Affine> t_kinect_1_2 = t_base_kinect * t_base_1_2 * t_base_kinect.inverse();
  
  // calculate transform between kinect and object in pose 2:
  Transform<float, 3, Affine> t_kinect_obj_2 = t_kinect_1_2.inverse() * t_kinect_obj_1;
  
  Transform<float, 3, Affine> t_world_obj(Translation<float,3>(0, 0, 0));

  cout << "Kinect to Kinect: " << endl << t_kinect_1_2.matrix() << endl
      << "Initial transform: " << endl << t_kinect_obj_1.matrix() << endl
      << "Final transform: " << endl << t_kinect_obj_2.matrix() << endl;

  // Theoretically, should be able to do the full chain to get the w->w transformation.
  // But not sure what the best way to do that is. So we'll figure it out.
  
  // Let's make some markers so we can visualize this better.
  ros::init(argc, argv, "transformation_test");
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("debug_markers", 10);
  ros::Rate r(1);
  
  visualization_msgs::Marker base_marker, kinect_marker, target_marker;
  base_marker.header.frame_id = kinect_marker.header.frame_id = target_marker.header.frame_id = "/odom";
  base_marker.header.stamp = kinect_marker.header.stamp = target_marker.header.stamp = ros::Time::now();
  
  Transform<float, 3, Affine> t_target_marker = t_world_obj;
  Transform<float, 3, Affine> t_kinect_marker = t_kinect_obj_1.inverse()*t_world_obj;
  Transform<float, 3, Affine> t_base_marker = t_base_kinect.inverse()*t_kinect_marker;
  
  cout << "Target marker: " << endl << t_target_marker.matrix() << endl
    << "Kinect marker: " << endl << t_kinect_marker.matrix() << endl
    << "Base marker: " << endl << t_base_marker.matrix() << endl
    << t_base_marker.translation().x() << endl;

  // Base is going to be a fat cylinder
  base_marker.id = 0;
  base_marker.type = visualization_msgs::Marker::CYLINDER;
  
  base_marker.scale.x = 0.3;
  base_marker.scale.y = 0.3;
  base_marker.scale.z = 0.05;
  
  base_marker.color.r = 1.0f;
  base_marker.color.g = 1.0f;
  base_marker.color.b = 1.0f;
  base_marker.color.a = 1.0;


  base_marker.pose.position.x = t_base_marker.translation().x();
  base_marker.pose.position.y = t_base_marker.translation().y();
  base_marker.pose.position.z = t_base_marker.translation().z();
  
  
  // Kinect is a box thing
  kinect_marker.id = 1;
  kinect_marker.type = visualization_msgs::Marker::CUBE;
  
  kinect_marker.scale.x = 0.025;
  kinect_marker.scale.y = 0.1;
  kinect_marker.scale.z = 0.025;
  
  kinect_marker.color.r = 0.0f;
  kinect_marker.color.g = 1.0f;
  kinect_marker.color.b = 1.0f;
  kinect_marker.color.a = 1.0;

  
  kinect_marker.pose.position.x = t_kinect_marker.translation().x();
  kinect_marker.pose.position.y = t_kinect_marker.translation().y();
  kinect_marker.pose.position.z = t_kinect_marker.translation().z();
  
  // Target is a... box thing
  target_marker.id = 2;
  target_marker.type = visualization_msgs::Marker::CUBE;
  
  target_marker.scale.x = 0.01;
  target_marker.scale.y = 0.25;
  target_marker.scale.z = 0.5;
  
  target_marker.color.r = 1.0f;
  target_marker.color.g = 1.0f;
  target_marker.color.b = 1.0f;
  target_marker.color.a = 1.0;

  
  target_marker.pose.position.x = t_target_marker.translation().x();
  target_marker.pose.position.y = t_target_marker.translation().y();
  target_marker.pose.position.z = t_target_marker.translation().z();
  
  while (ros::ok())
  {
    marker_pub.publish(base_marker);
    marker_pub.publish(kinect_marker);
    marker_pub.publish(target_marker);
    r.sleep();
  }

  
  /* Euler angles:
  Matrix3f m;
  m = AngleAxisf(angle1, Vector3f::UnitZ())
    * AngleAxisf(angle2, Vector3f::UnitY())
    * AngleAxisf(angle3, Vector3f::UnitZ());
  */

  /* Matrix3f m = Matrix3f::Random();
  m = (m + Matrix3f::Constant(1.2)) * 50;
  cout << "m =" << endl << m << endl;
  Vector3f v(1,2,3);
  
  cout << "m * v =" << endl << m * v << endl; */
}


#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry> 

#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>

#include <turtlebot_extrinsic_calibration/estimate_kinect_transform.h>

using namespace Eigen;
using namespace std;

using namespace visualization_msgs;

// To do: split into header file
class InteractiveTransformServer
{
  public:
    InteractiveTransformServer()
      : server("interactive_transform_test")
    {
      createTransforms();
      createInteractiveMarkers();
      marker_pub = nh.advertise<visualization_msgs::Marker>("debug_markers", 10);
    }
    
    void processFeedback(const InteractiveMarkerFeedbackConstPtr &feedback);
  
  private:
    void createInteractiveMarkers();
    void createTransforms();
    void updateTransforms();
  
    ros::NodeHandle nh;
    ros::Publisher marker_pub;
    interactive_markers::InteractiveMarkerServer server;
    Transform<float, 3, Affine> t_base_kinect, t_base_1_2, t_kinect_obj_1,
        t_kinect_1_2, t_kinect_obj_2, t_world_base;
        
    // Do I need these?
    Transform<float, 3, Affine> t_base_marker, t_target_marker, t_kinect_marker;
    
    // Visualization markers
    Marker base_marker, kinect_marker, target_marker;
    
    EstimateKinectTransform est;
};

void InteractiveTransformServer::createTransforms()
{
  // define transform between baselink and kinect.
  t_base_kinect = (Translation<float,3>(-.1, 0, .025));
  
  t_base_kinect = (Translation<float,3>(0, 0, 0));
  
  t_base_kinect = AngleAxis<float>(3.14/4.0, Vector3f(0,0,1));
  
  
  // define transform between baselink pose 1 and baselink pose 2:
  t_base_1_2 = (Translation<float,3>(-1, 0, 0));
  
  // define transform between kinect and object in pose 1:
  t_kinect_obj_1 = (Translation<float,3>(2, 0, 0));
  
  t_world_base = (Translation<float,3>(0, 0, 0));

  /* cout << "Kinect to Kinect: " << endl << t_kinect_1_2.matrix() << endl
      << "Initial transform: " << endl << t_kinect_obj_1.matrix() << endl
      << "Final transform: " << endl << t_kinect_obj_2.matrix() << endl; */
}

void InteractiveTransformServer::updateTransforms()
{ 
  /*t_kinect_marker = t_base_kinect*t_base_marker;
  t_kinect_1_2 = t_base_kinect * t_base_marker * t_base_kinect.inverse();
  t_kinect_obj_2 = t_kinect_obj_1*t_kinect_1_2.inverse();
  t_target_marker = t_base_marker.inverse()*t_base_kinect.inverse()*t_kinect_obj_2.inverse();*/
  
  
  // The kinect marker (world->kinect): world->base(new), base->kinect
  // Think about kinect->world vs world->kinect: which makes sense in this case?
  // I suppose this is never used, so doesn't matter.
  t_kinect_marker = t_base_kinect*t_base_marker;
  // The target marker (initially) (target(old)->world): target(old)->kinect, kinect->base(old), base(old)->world
  t_target_marker = t_world_base.inverse()*t_base_kinect.inverse()*t_kinect_obj_1.inverse();
  // Kinect(old) to kinect(new): (kinect(old)->kinect(new)): kinect->base, base(old)->base(new), base->kinect
  t_kinect_1_2 = t_base_kinect * t_base_marker * t_base_kinect.inverse();
  // View of target (currently) (kinect(new)->target(new)): kinect(new)->kinect(old), kinect(old)->target(old) 
  // (Does this make sense?) 
  t_kinect_obj_2 = t_kinect_obj_1*t_kinect_1_2.inverse();
  
  // The target marker (current) (target(new)->world): target(new)->kinect, kinect->base(new), base(new)->world
  t_target_marker = t_base_marker.inverse()*t_base_kinect.inverse()*t_kinect_obj_2.inverse();
  
  // To project: (target(new)->kinect)
  t_kinect_obj_2.inverse();
  
  Transform<float, 3, Affine> t_obj_1_2 = t_kinect_obj_1 * t_kinect_obj_2.inverse();
  
  //cout << "Kinect to Kinect: " << endl << t_kinect_1_2.matrix() << endl
    //<< "Object to Object: " << endl << t_obj_1_2.matrix() << endl
    //<< "Baselink: " << endl << t_base_marker.matrix() << endl
    //<< "Kinect marker: " << endl << t_kinect_marker.matrix() << endl
    //<< "Initial transform: " << endl << t_kinect_obj_1.matrix() << endl
    //<< "Final transform: " << endl << t_kinect_obj_2.matrix() << endl
    //<< "Target marker: " << endl << t_target_marker.matrix() << endl;
}

void InteractiveTransformServer::processFeedback(
    const InteractiveMarkerFeedbackConstPtr &feedback )
{     
  t_base_marker = Translation<float,3>(feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z);
  
  // How confident am I that it's a prerotate? Should take a point (0,0,0) through t_base_marker and see what happens...
  t_base_marker.prerotate(Quaternion<float>(feedback->pose.orientation.w, feedback->pose.orientation.x, feedback->pose.orientation.y, feedback->pose.orientation.z));

  updateTransforms();
      
  Vector3f target_center(0, .125, .25);
  Vector3f target_center_transformed = t_target_marker*target_center;
  
  //cout << "Target center: " << target_center_transformed << endl;
      
  // WTFFFF HAXXXX ******
  target_marker.pose.position.x = -target_center_transformed.x();
  target_marker.pose.position.y = -target_center_transformed.y();
  target_marker.pose.position.z = -target_center_transformed.z();
  
  Quaternion<float> quat(t_target_marker.rotation());
  
  target_marker.pose.orientation.x = quat.x();
  target_marker.pose.orientation.y = quat.y();
  target_marker.pose.orientation.z = quat.z();
  target_marker.pose.orientation.w = quat.w();
      
  marker_pub.publish(target_marker);
  
  switch ( feedback->event_type )
  {
    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
      est.addData(t_base_marker, t_kinect_obj_2);
      est.computeTransform();
      cout << "True marker position: " << endl << t_target_marker.inverse().matrix() << endl;
      cout << "Result transform: " << endl << est.getTransform().matrix() << endl;
      cout << "Actual transform: " << endl << t_base_kinect.matrix() << endl;
      break;
    default:
      //est.addData(t_base_marker, t_kinect_obj_2); 
      break;
  }
    
  //server.applyChanges();
}

void InteractiveTransformServer::createInteractiveMarkers()
{
  // create an interactive marker for our server
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "/odom";
  int_marker.name = "turtlebot_marker";
  int_marker.description = "Move the turtlebot";

  base_marker.header.frame_id = kinect_marker.header.frame_id = target_marker.header.frame_id = "/odom";
  base_marker.header.stamp = kinect_marker.header.stamp = target_marker.header.stamp = ros::Time::now();
  
  /* Transform<float, 3, Affine> t_target_marker = t_world_obj;
  Transform<float, 3, Affine> t_kinect_marker = t_kinect_obj_1.inverse()*t_world_obj;
  Transform<float, 3, Affine> t_base_marker = t_base_kinect.inverse()*t_kinect_marker; */

  t_base_marker = t_world_base;
  t_kinect_marker = t_base_kinect*t_base_marker;
  t_target_marker = t_kinect_marker.inverse()*t_kinect_obj_1.inverse();
  
  //cout << "Target marker: " << endl << t_target_marker.matrix() << endl
  //  << "Kinect marker: " << endl << t_kinect_marker.matrix() << endl
  //  << "Base marker: " << endl << t_base_marker.matrix() << endl
   // << t_base_marker.translation().x() << endl;

  // Base is going to be a fat cylinder
  base_marker.id = 0;
  base_marker.type = visualization_msgs::Marker::CYLINDER;
  
  base_marker.scale.x = 0.3;
  base_marker.scale.y = 0.3;
  base_marker.scale.z = 0.05;
  
  base_marker.color.r = 1.0f;
  base_marker.color.g = 1.0f;
  base_marker.color.b = 1.0f;
  base_marker.color.a = 0.5;


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
  
  Quaternion<float> quat(t_kinect_marker.inverse().rotation());
  
  kinect_marker.pose.orientation.x = quat.x();
  kinect_marker.pose.orientation.y = quat.y();
  kinect_marker.pose.orientation.z = quat.z();
  kinect_marker.pose.orientation.w = quat.w();
      
  
  // Target is a... box thing
  target_marker.id = 0;
  target_marker.type = visualization_msgs::Marker::CUBE;
  
  target_marker.scale.x = 0.01;
  target_marker.scale.y = 0.25;
  target_marker.scale.z = 0.5;
  
  target_marker.color.r = 1.0f;
  target_marker.color.g = 1.0f;
  target_marker.color.b = 1.0f;
  target_marker.color.a = 1.0;

  // create a non-interactive control which contains the box
  InteractiveMarkerControl box_control;
  box_control.always_visible = true;
  box_control.markers.push_back( base_marker );
  box_control.markers.push_back( kinect_marker );

  // add the control to the interactive marker 
  int_marker.controls.push_back( box_control );
  
  InteractiveMarkerControl control;

  //control.orientation_mode = InteractiveMarkerControl::FIXED;
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
  
  control.interaction_mode = InteractiveMarkerControl::MOVE_ROTATE;
  //control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  
  // add the interactive marker to our collection &
  // tell the server to call processFeedback() when feedback arrives for it
  server.insert(int_marker, boost::bind( &InteractiveTransformServer::processFeedback, this, _1 ));
  
  // 'commit' changes and send to all clients
  server.applyChanges();
  
}



int main( int argc, char** argv )
{
  ros::init(argc, argv, "interactive_transform_test");
  InteractiveTransformServer transform_test;
  
  // start the ROS main loop
  ros::spin();
}


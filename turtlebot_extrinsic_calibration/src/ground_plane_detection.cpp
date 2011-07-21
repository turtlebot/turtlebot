#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

using namespace std;

ros::Publisher pub;

void callback(const PointCloud::ConstPtr& msg)
{
  printf ("Cloud: width = %d, height = %d, size: %d\n", msg->width, msg->height,  msg->width*msg->height);
  
  PointCloud cloud_filtered, cloud_ground;
  
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud (msg);
  sor.setLeafSize (0.02, 0.02, 0.02);
  sor.filter (cloud_filtered);

  std::cerr << "PointCloud after filtering: " << cloud_filtered.width * cloud_filtered.height 
       << " data points (" << pcl::getFieldsList (cloud_filtered) << ").\n";

  // Let's estimate us some normals!
  
  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (cloud_filtered.makeShared());

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ> ());
  ne.setSearchMethod (tree);

  // Output datasets
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

  // Use all neighbors in a sphere of radius 3cm
  ne.setRadiusSearch (0.03);

  // Compute the features
  ne.compute (*cloud_normals);



  
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);
  seg.setAxis(Eigen::Vector3f(0,1,0));
  seg.setProbability(0.5);
  
  cout << "Probability: " << seg.getProbability() << endl;


 /* for (size_t i = 0; i < inliers->indices.size (); ++i)
    std::cerr << inliers->indices[i] << "    " << cloud.points[inliers->indices[i]].x << " "
                                               << cloud.points[inliers->indices[i]].y << " "
                                               << cloud.points[inliers->indices[i]].z << std::endl; */
  
  //BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points)
  //  printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
         
         
    // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  pcl::ExtractIndices<pcl::Normal> extract_normals;

  bool found_ground = false;
  int i = 0, nr_points = cloud_filtered.points.size ();

  while (!found_ground && cloud_filtered.points.size () > 0.3 * nr_points)
  {
    seg.setInputCloud (cloud_filtered.makeShared ());
    seg.setInputNormals (cloud_normals);
    
    cout << "Cloud filtered size: " << cloud_filtered.size() << " Cloud normals size: " << cloud_normals->size() << endl;
    
    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0)
    {
      PCL_ERROR ("Could not estimate a planar model for the given dataset.");
      return;
    }

    
  
    if (inliers->indices.size () == 0)
    {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
        break;
    }
    
    // Extract the inliers
    extract.setInputCloud (cloud_filtered.makeShared());
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (cloud_ground);
    
    Eigen::Vector3f floor_normal(0,1,0);
    Eigen::Vector3f plane_normal(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
    
    
    
    if (floor_normal.dot(plane_normal) > 0.5)
    {
      found_ground = true;
      

      
      std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                        << coefficients->values[1] << " "
                                        << coefficients->values[2] << " " 
                                        << coefficients->values[3] << std::endl;

      std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
      
      if (cloud_ground.size() < nr_points/10.0)
      {
        std::cerr << "PointCloud " << i << " with normal agreement " << floor_normal.dot(plane_normal) << std::endl;

        cout << "But not using this one because it sucks... try again!" << endl;
        found_ground = false;
      }
      else
        pub.publish(cloud_ground);
    }

    extract.setNegative (true);
    extract.filter (cloud_filtered);
    
    extract_normals.setInputCloud (cloud_normals);
    extract_normals.setIndices (inliers);
    extract_normals.setNegative (true);
    extract_normals.filter (*cloud_normals);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ground_plane_detection");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<PointCloud>("/camera/rgb/points", 1, callback);
  pub = nh.advertise<PointCloud> ("ground_points", 1);
  ros::spin();
}

/*
int
 main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;

  // Fill in the cloud data
  cloud.width  = 15;
  cloud.height = 1;
  cloud.points.resize (cloud.width * cloud.height);

  // Generate the data
  for (size_t i = 0; i < cloud.points.size (); ++i)
  {
    cloud.points[i].x = 1024 * rand () / (RAND_MAX + 1.0);
    cloud.points[i].y = 1024 * rand () / (RAND_MAX + 1.0);
    cloud.points[i].z = 1.0;
  }

  // Set a few outliers
  cloud.points[0].z = 2.0;
  cloud.points[3].z = -2.0;
  cloud.points[6].z = 4.0;

  std::cerr << "Point cloud data: " << cloud.points.size () << " points" << std::endl;
  for (size_t i = 0; i < cloud.points.size (); ++i)
    std::cerr << "    " << cloud.points[i].x << " " 
                        << cloud.points[i].y << " " 
                        << cloud.points[i].z << std::endl;

  

  return (0);
}
*/


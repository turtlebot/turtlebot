#include <turtlebot_extrinsic_calibration/detect_ground_plane.h>

using namespace std;

bool GroundPlaneDetector::findGroundPlane(const PointCloud::ConstPtr& msg)
{
  printf ("Cloud: width = %d, height = %d, size: %d\n", msg->width, msg->height,  msg->width*msg->height);
  
  pcl::PointCloud<pcl::PointXYZ> cloud_filtered, cloud_ground;
  
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud (msg);
  sor.setLeafSize (leaf_size, leaf_size, leaf_size);
  sor.filter (cloud_filtered);

  std::cerr << "PointCloud after filtering: " << cloud_filtered.width * cloud_filtered.height 
       << " data points (" << pcl::getFieldsList (cloud_filtered) << ").\n";

  // Estimate normals
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (cloud_filtered.makeShared());
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ> ());
  ne.setSearchMethod (tree);
  ne.setRadiusSearch (normal_radius);
  ne.compute (*cloud_normals);


  // Segment ground out using SACSegmentationFromNormals
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  
  // Create the segmentation object
  pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (distance_threshold);
  seg.setAxis(normal);
         
  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  pcl::ExtractIndices<pcl::Normal> extract_normals;

  int i = 0, nr_points = cloud_filtered.points.size ();

  while (cloud_filtered.points.size () > min_percentage_ground * nr_points)
  {
    seg.setInputCloud (cloud_filtered.makeShared ());
    seg.setInputNormals (cloud_normals);
    
    cout << "Cloud filtered size: " << cloud_filtered.size() << " Cloud normals size: " << cloud_normals->size() << endl;
    
    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0)
    {
      PCL_ERROR ("Could not estimate a planar model for the given dataset.");
      return false;
    }
    
    // Extract the inliers
    extract.setInputCloud (cloud_filtered.makeShared());
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (cloud_ground);
    
    Eigen::Vector3f plane_normal(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
    
    
    // Two conditions: the dot product between the normals is above a certain 
    // threshold AND it's at least the minimum percentage ground
    if (normal.dot(plane_normal) > normal_threshold 
        && cloud_ground.size() > nr_points*min_percentage_ground)
    {
      std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                        << coefficients->values[1] << " "
                                        << coefficients->values[2] << " " 
                                        << coefficients->values[3] << std::endl;
      std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
      std::cerr << "PointCloud " << i << " with normal agreement " << normal.dot(plane_normal) << std::endl;
      
      cloud_output = cloud_ground;
      return true;
    }

    extract.setNegative (true);
    extract.filter (cloud_filtered);
    
    extract_normals.setInputCloud (cloud_normals);
    extract_normals.setIndices (inliers);
    extract_normals.setNegative (true);
    extract_normals.filter (*cloud_normals);
  }
  
  return false;
};


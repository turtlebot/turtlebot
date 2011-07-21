#ifndef _DETECT_GROUND_PLANE__
#define _DETECT_GROUND_PLANE_

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

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class GroundPlaneDetector
{
  public:
    double leaf_size;
    double normal_radius;
    double probability;
    double distance_threshold;
    
    double normal_threshold; // minimum agreement between expected and found normal
    double min_percentage_ground; // minimum percentage the ground plane must comprise
    
    Eigen::Vector3f normal;
    
    Eigen::Vector3f detected_normal;
    double detected_distance;
    
    pcl::PointCloud<pcl::PointXYZ> cloud_output;
    
  public:
    GroundPlaneDetector()
      : leaf_size(0.02), normal_radius(0.1), probability(0.5), 
        distance_threshold(0.1), normal_threshold(0.9), 
        min_percentage_ground(0.1), normal(0,1,0)
    { }
  
    void setExpectedNormal(Eigen::Vector3f normal_) { normal = normal_; }
    
    // Returns whether a ground plane was found
    bool findGroundPlane(const PointCloud::ConstPtr& msg);

    double getDistance() { return detected_distance; }
    Eigen::Vector3f getNormal() { return detected_normal; }
};

#endif

#ifndef _DETECT_CALIBRATION_PATTERN_
#define _DETECT_CALIBRATION_PATTERN_

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry> 
#include <Eigen/StdVector>

#include <iostream>
#include <stdexcept>

using namespace std;

enum Pattern
{
  CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID
};

typedef std::vector<cv::Point3f> object_pts_t;
typedef std::vector<cv::Point2f> observation_pts_t;

void convertCVtoEigen(cv::Mat& tvec, cv::Mat& R, Eigen::Vector3f& translation, Eigen::Quaternionf& orientation);

class PatternDetector
{
  public:
    PatternDetector() { }
  
    static object_pts_t calcChessboardCorners(cv::Size boardSize,
                                          float squareSize,
                                          Pattern patternType = CHESSBOARD,
                                          cv::Point3f offset = cv::Point3f());
                                          
    int detectPattern(cv::Mat& inm, Eigen::Vector3f& translation, Eigen::Quaternionf& orientation);
    
    void setCameraMatrices(cv::Mat K_, cv::Mat D_);
    
    void setPattern(cv::Size grid_size_, float square_size_, 
          Pattern pattern_type_, cv::Point3f offset_ = cv::Point3f());

  public:
    cv::Mat K;
    cv::Mat D;
    cv::Mat rvec, tvec, R;
    
    // Do we need to store anything but type and ideal points?
    // Who knows!
    Pattern pattern_type;
    cv::Size grid_size;
    float square_size;
    object_pts_t ideal_points;
};

#endif

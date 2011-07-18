#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <stdexcept>

using namespace std;

enum Pattern
{
  CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID
};

typedef std::vector<cv::Point3f> object_pts_t;
typedef std::vector<cv::Point2f> observation_pts_t;

class PatternDetector
{
  public:
    static object_pts_t calcChessboardCorners(cv::Size boardSize,
                                          float squareSize,
                                          Pattern patternType = CHESSBOARD,
                                          cv::Point3f offset = cv::Point3f());
                                          
    void detectPattern(const cv::Mat& inm);
    
    void setCameraMatrices(cv::Mat K_, cv::Mat D_);
    
    
    void getPose();
    void getPoints();
    void setPattern(cv::Size grid_size_, float square_size_, 
          Pattern pattern_type_, cv::Point3f offset_ = cv::Point3f());

  private:
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

void PatternDetector::setCameraMatrices(cv::Mat K_, cv::Mat D_)
{
  K = K_;
  D = D_; 
}

void PatternDetector::setPattern(cv::Size grid_size_, float square_size_, 
      Pattern pattern_type_, cv::Point3f offset_)
{
  ideal_points = calcChessboardCorners(grid_size_, square_size_, pattern_type_, offset_);
  pattern_type = pattern_type_;
  grid_size = grid_size_;
  square_size = square_size_;
}

object_pts_t PatternDetector::calcChessboardCorners(cv::Size boardSize,
                                          float squareSize,
                                          Pattern patternType,
                                          cv::Point3f offset)
{
  object_pts_t corners;
  switch (patternType)
  {
    case CHESSBOARD:
    case CIRCLES_GRID:
      for (int i = 0; i < boardSize.height; i++)
        for (int j = 0; j < boardSize.width; j++)
          corners.push_back(
                            cv::Point3f(float(j * squareSize),
                                        float(i * squareSize), 0) + offset);
      break;
    case ASYMMETRIC_CIRCLES_GRID:
      for (int i = 0; i < boardSize.height; i++)
        for (int j = 0; j < boardSize.width; j++)
          corners.push_back(
                            cv::Point3f(float(i * squareSize),
                                        float((2 * j + i % 2) * squareSize), 0) + offset);
      break;
    default:
      std::logic_error("Unknown pattern type.");
  }
  return corners;
}



void PatternDetector::detectPattern(const cv::Mat& inm)
{
  // Set these according to known quantities...
  bool found = false;
  
  // What is outv? inm?
  observation_pts_t observation_points;
  
  switch (pattern_type)
  {
    case ASYMMETRIC_CIRCLES_GRID:
      found = cv::findCirclesGrid(inm, grid_size, observation_points,
                                cv::CALIB_CB_ASYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING);
      break;
    case CHESSBOARD:
      found = cv::findChessboardCorners(inm, grid_size, observation_points);
      break;
    case CIRCLES_GRID:
      found = cv::findCirclesGrid(inm, grid_size, observation_points, cv::CALIB_CB_SYMMETRIC_GRID);
      break;
  }

  if(found)
  {
    cv::solvePnP(cv::Mat(ideal_points), cv::Mat(observation_points), K, D,
                 rvec, tvec, false);
    cv::Rodrigues(rvec, R); //take the 3x1 rotation representation to a 3x3 rotation matrix.
  }
  
  cout << "Found? " << found << endl;
}

/*int main()
{
}*/

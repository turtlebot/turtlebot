#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

enum Pattern
{
  CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID
};

typedef std::vector<cv::Point3f> object_pts_t;
typedef std::vector<cv::Point2f> observation_pts_t;

static object_pts_t calcChessboardCorners(cv::Size boardSize,
                                          float squareSize,
                                          Pattern patternType = CHESSBOARD,
                                          cv::Point3f offset = cv::Point3f())
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



void pattern_detection()
{
  // Set these according to known quantities...
  cv::Size grid_size_;
  float square_size_;
  Pattern pattern_;
  object_pts_t ideal_pts_;
  ideal_pts_ = calcChessboardCorners(grid_size_, square_size_, pattern_,offset);
  bool found = false;
  switch (pattern_)
  {
    case ASYMMETRIC_CIRCLES_GRID:
      found
          = cv::findCirclesGrid(inm, grid_size_, outv,
                                cv::CALIB_CB_ASYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING);
      break;
    case CHESSBOARD:
      found = cv::findChessboardCorners(inm, grid_size_,
                                                          outv);
      break;
    case CIRCLES_GRID:
      found
          = cv::findCirclesGrid(inm, grid_size_, outv,
                                cv::CALIB_CB_SYMMETRIC_GRID);
      break;
  }

  if(found)
  {
    // Set these according to the camera matrix broadcast.
    
    cv::Mat K = ...; //3x3 camera matrix.
    cv::Mat D;//assume no distortion
    cv::Mat rvec, tvec,R;//output variables.
    cv::solvePnP(object_points, observation_points, K, D,
                 rvec, tvec,
                 false);
    cv::Rodrigues(rvec, R); //take the 3x1 rotation representation to a 3x3 rotation matrix.
  }
}

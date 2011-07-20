#include <turtlebot_extrinsic_calibration/detect_calibration_pattern.h>

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



int PatternDetector::detectPattern(cv::Mat& inm, Eigen::Vector3f& translation, Eigen::Quaternionf& orientation)
{
  translation.setZero();
  orientation.setIdentity();
  
  // Set these according to known quantities...
  bool found = false;
  
  // What is outv? inm?
  observation_pts_t observation_points;
  
  switch (pattern_type)
  {
    case ASYMMETRIC_CIRCLES_GRID:
      //found = cv::findCirclesGrid(inm, grid_size, observation_points,
      //                          cv::CALIB_CB_ASYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING);
      break;
    case CHESSBOARD:
      //cout << "Finding chessboard in image with " << inm.total() << " pixels " << endl;
      found = cv::findChessboardCorners(inm, grid_size, observation_points, cv::CALIB_CB_ADAPTIVE_THRESH);
      break;
    case CIRCLES_GRID:
      //found = cv::findCirclesGrid(inm, grid_size, observation_points, cv::CALIB_CB_SYMMETRIC_GRID);
      break;
  }

  if(found)
  {
    cv::solvePnP(cv::Mat(ideal_points), cv::Mat(observation_points), K, D,
                 rvec, tvec, false);
    cv::Rodrigues(rvec, R); //take the 3x1 rotation representation to a 3x3 rotation matrix.
    
    cv::drawChessboardCorners(inm, grid_size, cv::Mat(observation_points), found);
    
    convertCVtoEigen(tvec, R, translation, orientation);
  }
  
  cout << "Found? " << found << endl;
  
  return found;
}

void convertCVtoEigen(cv::Mat& tvec, cv::Mat& R, Eigen::Vector3f& translation, Eigen::Quaternionf& orientation)
{
  // This assumes that cv::Mats are stored as doubles. Is there a way to check this?
  // Since it's templated...
  translation = Eigen::Vector3f(tvec.at<double>(0,0), tvec.at<double>(0,1), tvec.at<double>(0, 2));
  
  Eigen::Matrix3f Rmat;
  Rmat << R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),
          R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),
          R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2);
                                          
  orientation = Eigen::Quaternionf(Rmat);

}


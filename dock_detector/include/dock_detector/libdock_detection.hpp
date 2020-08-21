#pragma once

#include <ros/ros.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <utility>
#include <vector>

namespace ariitk::dock_detection
{
class DockDetector
{
public:
  std::pair<int, int> getCentre();
  cv::Mat getThresh();
  // cv::Scalar circleDet(const std::vector<cv::Point>& contour); //Using this variant implementing like that of
  // helipad_det.
  cv::Scalar circleDet(cv::Mat& img);  // Using this variant implementing hough tranform.
  double getDistance();

  void setHSVMin(const int& h, const int& s, const int& v);
  void setHSVMax(const int& h, const int& s, const int& v);
  void setMinArea(const int& area);
  void setCannyParams(const int& lower, const int& upper, const int& size);
  void setInterCentreDist(const double& distance);

  void thresholdImage(cv::Mat& img);
  void findAllContours(cv::Mat& img);
  void drawContours(cv::Mat& img);

private:
  std::pair<int, int> centre_;
  cv::Scalar hsv_min_;
  cv::Scalar hsv_max_;

  cv::Mat thresh_img_;
  cv::Mat canny_img_;

  cv::Point2f center_;

  std::vector<std::vector<cv::Point>> all_contours_;

  int canny_param_lower_;
  int canny_param_upper_;
  int canny_kernel_size_;

  double min_contour_area_;
  double distance_;
  double area_;
  double inter_centre_dist_;
};

}  // namespace ariitk::dock_detection
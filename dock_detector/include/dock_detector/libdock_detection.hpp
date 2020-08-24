#pragma once

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <ros/ros.h>
#include <utility>
#include <vector>

namespace ariitk::dock_detection {
class DockDetector {
  public:
    std::pair<int, int> getCentre(cv::Mat& img, const double& std_dev_bound);
    cv::Mat getThresh();
    cv::Scalar circleDet(const std::vector<cv::Point>& contour, const double& std_dev_bound);
    double getDistance();

    void setHSVMin(const int& h, const int& s, const int& v);
    void setHSVMax(const int& h, const int& s, const int& v);
    void setMinArea(const int& area);
    void setCannyParams(const int& lower, const int& upper, const int& size);

    void thresholdImage(cv::Mat& img);
    void findGoodContours(cv::Mat& img);
    void drawContours(cv::Mat& img);

  private:
    std::pair<int, int> centre_;
    cv::Scalar hsv_min_;
    cv::Scalar hsv_max_;

    cv::Mat thresh_img_;

    cv::Point2f center_;

    std::vector<std::vector<cv::Point>> good_contours_;

    int canny_param_lower_;
    int canny_param_upper_;
    int canny_kernel_size_;

    double min_contour_area_;
    double distance_;
};

}  // namespace ariitk::dock_detection
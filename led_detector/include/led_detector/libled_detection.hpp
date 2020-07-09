#pragma once

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <utility>
#include <vector>

namespace iarc2020::led_detection {

class LedDetector {
  public:
    cv::Point2f getCentreRed();
    cv::Point2f getCentreGreen();
    cv::Point2f getCentreBlue();
    cv::Mat getThreshRed();
    cv::Mat getThreshGreen();
    double getDistanceRed();
    double getDistanceGreen();

    void setHSVMinRed(const int& h, const int& s, const int& v);
    void setHSVMaxRed(const int& h, const int& s, const int& v);
    void setHSVMinGreen(const int& h, const int& s, const int& v);
    void setHSVMaxGreen(const int& h, const int& s, const int& v);
    void setMaxArea(const int& area);
    void setCannyParams(const int& lower, const int& upper, const int& size);

    void thresholdImage(cv::Mat& img);
    void findGoodContours();
    void fitCircle();
    void drawContours(cv::Mat& img);
    void findFrameCentre(cv::Mat& img);
    void fitRect(cv::Mat& img);

    static double scale_factor_red;
    static double scale_factor_green;

  private:
    cv::Point2f centre_red_;
    cv::Point2f centre_green_;

    cv::Scalar hsv_min_red_;
    cv::Scalar hsv_max_red_;
    cv::Scalar hsv_min_green_;
    cv::Scalar hsv_max_green_;

    cv::Mat thresh_img_red_;
    cv::Mat thresh_img_green_;

    std::vector<std::vector<cv::Point>> contours_red_;
    std::vector<std::vector<cv::Point>> contours_green_;

    int canny_param_lower_;
    int canny_param_upper_;
    int canny_kernel_size_;

    double max_contour_area_;
    double distance_red_;
    double distance_green_;
    double area_;
};

}  // namespace iarc2020::led_detection

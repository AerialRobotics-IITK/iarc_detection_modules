#pragma once

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <utility>
#include <vector>

namespace iarc2020::plate_detector {

class PlateDetector {
    public:
    ~PlateDetector() { cv::destroyAllWindows(); };

    std::pair<int, int> getCentre() { return centre_; };
    cv::Mat getThresh() { return thresh_img_; };
    double getDistance() { return distance_; };

    void setHSVMin(const int& h, const int& s, const int& v) { hsv_min_ = cv::Scalar(h, s, v); }
    void setHSVMax(const int& h, const int& s, const int& v) { hsv_max_ = cv::Scalar(h, s, v); }
    void setMinArea(const int& area) { min_contour_area_ = area; }
    void setCannyParams(const int& lower, const int& upper, const int& size);

    void thresholdImage(cv::Mat& img);
    void findGoodContours();
    void drawContours(cv::Mat& img);
    void findFrameCentre(cv::Mat& img);
    void fitRect(cv::Mat& img);

    static double scale_factor;

    private:
    std::pair<int, int> centre_;

    cv::Scalar hsv_min_;
    cv::Scalar hsv_max_;

    cv::Mat thresh_img_;

    cv::Point2f center_;
    cv::Point2f rect_points[4];

    std::vector<std::vector<cv::Point>> good_contours_;

    int canny_param_lower_;
    int canny_param_upper_;
    int canny_kernel_size_;

    double min_contour_area_;
    double distance_;
};

}  // namespace iarc2020::plate_detector

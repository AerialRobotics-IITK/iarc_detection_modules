#include <led_detector/libled_detection.hpp>

namespace iarc2020::led_detection {

double LedDetector::scale_factor = 117;  // FIXME: why 123?

void LedDetector::setCannyParams(const int& lower, const int& upper, const int& size) {
    canny_param_lower_ = lower;
    canny_param_upper_ = upper;
    canny_kernel_size_ = size;
}

void LedDetector::thresholdImage(cv::Mat& img) {
    if (img.empty()) { return; }

    cv::Mat blur_img(img.size(), CV_8UC3);

    cv::GaussianBlur(img, blur_img, cv::Size(3, 3), 0, 0);
    cv::inRange(blur_img, hsv_min_red_, hsv_max_red_, thresh_img_red_);
    cv::inRange(blur_img, hsv_min_green_, hsv_max_green_, thresh_img_green_);
}

void LedDetector::findGoodContours() {
    cv::Mat canny_img_red(thresh_img_red_.size(), CV_8UC1);
    cv::Mat canny_img_green(thresh_img_green_.size(), CV_8UC1);

    cv::Canny(thresh_img_red_, canny_img_red, canny_param_lower_, canny_param_upper_, canny_kernel_size_);
    cv::Canny(thresh_img_green_, canny_img_green, canny_param_lower_, canny_param_upper_, canny_kernel_size_);

    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(thresh_img_red_, contours_red_, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
    cv::findContours(thresh_img_green_, contours_green_, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

    centre_red_.x = -1;
    centre_red_.y = -1;
    centre_green_.x = -1;
    centre_green_.y = -1;

    float temp_radius;

    if(!contours_red_.empty()) {
        if (cv::contourArea(contours_red_[0]) < max_contour_area_) { cv::minEnclosingCircle(contours_red_[0], centre_red_, temp_radius); }
        contours_red_.clear();
    }
    if(!contours_green_.empty()) {
        if (cv::contourArea(contours_green_[0]) < max_contour_area_) { cv::minEnclosingCircle(contours_green_[0], centre_green_, temp_radius); }
        contours_red_.clear();
    }
}

void LedDetector::drawContours(cv::Mat& board) {
    cv::drawContours(board, contours_red_, -1, cv::Scalar(255, 255, 255), 2);
    cv::drawContours(board, contours_green_, -1, cv::Scalar(255, 255, 255), 2);
    cv::circle(board, centre_red_, 5, cv::Scalar(255, 0, 0));
    cv::circle(board, centre_green_, 5, cv::Scalar(255, 0, 0));
}

}  // namespace iarc2020::led_detection

#include <plate_detector/libplate_detection.hpp>

namespace ariitk::plate_detection {

void PlateDetector::setCannyParams(const int& lower, const int& upper, const int& size) {
    canny_param_lower_ = lower;
    canny_param_upper_ = upper;
    canny_kernel_size_ = size;
}

void PlateDetector::thresholdImage(cv::Mat& img) {
    if (img.empty()) {
        return;
    }

    cv::Mat blur_img(img.size(), CV_8UC3);
    cv::Mat hsv_img(img.size(), CV_8UC3);

    cv::GaussianBlur(img, blur_img, cv::Size(3, 3), 0, 0);
    cv::cvtColor(blur_img, hsv_img, CV_BGR2HSV);
    cv::inRange(hsv_img, hsv_min_, hsv_max_, thresh_img_);
}

void PlateDetector::findGoodContours() {
    cv::Mat canny_img(thresh_img_.size(), CV_8UC1);

    cv::Canny(thresh_img_, canny_img, canny_param_lower_, canny_param_upper_, canny_kernel_size_);

    good_contours_.clear();
    good_contours_.shrink_to_fit();
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(thresh_img_, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

    good_contours_.clear();
    for (auto& contour : contours) {
        if (cv::contourArea(contour) > min_contour_area_) {
            good_contours_.push_back(contour);
        }
    }
}

void PlateDetector::fitRect(cv::Mat& board) {
    int area_curr = 0;
    int area_prev = 0;
    center_.x = -1;
    center_.y = -1;
    distance_ = 0.0;

    for (int i = 0; i < good_contours_.size(); ++i) {
        if (good_contours_.empty()) {
            break;
        }
        cv::RotatedRect rotatedRect = cv::minAreaRect(good_contours_[i]);
        area_curr = rotatedRect.size.height * rotatedRect.size.width;
        if (area_curr > area_prev) {
            center_ = rotatedRect.center;
            rotatedRect.points(rect_points);
        }
        area_prev = area_curr;
    }
    centre_.first = center_.x;
    centre_.second = center_.y;

    /*
    * If you want to revert back to estimation using side use this:

    distance_ = sqrt((rect_points[1].x - rect_points[0].x) * (rect_points[1].x - rect_points[0].x) +
                     (rect_points[1].y - rect_points[0].y) * (rect_points[1].y - rect_points[0].y));

    distance_ = scale_factor / distance_;
    */

    area_ = sqrt((rect_points[1].x - rect_points[0].x) * (rect_points[1].x - rect_points[0].x) +
                 (rect_points[1].y - rect_points[0].y) * (rect_points[1].y - rect_points[0].y)) *
            sqrt((rect_points[2].x - rect_points[1].x) * (rect_points[2].x - rect_points[1].x) +
                 (rect_points[2].y - rect_points[1].y) * (rect_points[2].y - rect_points[1].y));

    distance_ = sqrt(scale_factor / area_);
    cv::circle(board, center_, 5, cv::Scalar(255, 0, 0));

    // std::cout << rect_points[0] << std::endl 
    //           << rect_points[1] << std::endl
    //           << rect_points[2] << std::endl
    //           << rect_points[3] << std::endl << std::endl;

}

void PlateDetector::drawContours(cv::Mat& board) {
    cv::drawContours(board, good_contours_, -1, cv::Scalar(255, 255, 255), 2);
}

std::pair<int, int> PlateDetector::getCentre() {
    return centre_;
}

cv::Mat PlateDetector::getThresh() {
    return thresh_img_;
}

cv::Point2f* PlateDetector::getCorners() {
    return rect_points;
}

double PlateDetector::getDistance() {
    return distance_;
}

double PlateDetector::getArea() {
    return area_;
}

void PlateDetector::setHSVMin(const int& h, const int& s, const int& v) {
    hsv_min_ = cv::Scalar(h, s, v);
}

void PlateDetector::setHSVMax(const int& h, const int& s, const int& v) {
    hsv_max_ = cv::Scalar(h, s, v);
}

void PlateDetector::setMinArea(const int& area) {
    min_contour_area_ = area;
}

}  // namespace ariitk::plate_detection

#include <dock_detector/libdock_detection.hpp>

namespace ariitk::dock_detection {
void DockDetector::setCannyParams(const int& lower, const int& upper, const int& size) {
    canny_param_lower_ = lower;
    canny_param_upper_ = upper;
    canny_kernel_size_ = size;
}

void DockDetector::thresholdImage(cv::Mat& img) {
    if (img.empty()) {
        return;
    }

    cv::Mat blur_img(img.size(), CV_8UC3);
    cv::Mat hsv_img(img.size(), CV_8UC3);
    cv::Mat dilated_img(hsv_img.size(), CV_8UC3);
    cv::Mat dilated_again(hsv_img.size(), CV_8UC3);
    cv::Mat kernel = cv::Mat::eye(5, 5, CV_8U);

    cv::GaussianBlur(img, blur_img, cv::Size(3, 3), 0, 0);
    cv::cvtColor(blur_img, hsv_img, CV_BGR2HSV);
    cv::dilate(hsv_img, dilated_img, kernel);
    cv::dilate(dilated_img, dilated_again, kernel);
    cv::inRange(dilated_again, hsv_min_, hsv_max_, thresh_img_);
}

void DockDetector::findGoodContours(cv::Mat& img) {
    cv::Mat canny_img(thresh_img_.size(), CV_8UC1);
    cv::Canny(thresh_img_, canny_img, canny_param_lower_, canny_param_upper_, canny_kernel_size_);

    good_contours_.clear();
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(thresh_img_, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

    for (auto& contour : contours) {
        if (cv::contourArea(contour) > min_contour_area_) {
            good_contours_.push_back(contour);
        }
    }
}

cv::Scalar DockDetector::circleDet(const std::vector<cv::Point>& contour, const double& std_dev_bound) {
    int n = contour.size();
    if (n < 10)
        return cv::Scalar(0, 0, 0);
    cv::Point centre = 0.25 * (contour.at(0) + contour.at(round(n / 2))) + 0.25 * (contour.at(round(n / 4)) + contour.at(round(3 * n / 4)));
    double avg_r2 = 0, avg_r = 0;
    for (int i = 0; i < n; i++) {
        avg_r += cv::norm(contour.at(i) - centre);
        avg_r2 += cv::norm(contour.at(i) - centre) * cv::norm(contour.at(i) - centre);
    }
    avg_r = avg_r / n;
    avg_r2 = avg_r2 / n;
    double standard_deviation = std::sqrt(avg_r2 - avg_r * avg_r);
    if (standard_deviation < std_dev_bound * avg_r) {
        return cv::Scalar(centre.x, centre.y, avg_r);
    } else {
        return cv::Scalar(0, 0, 0);
    }
}

void DockDetector::drawContours(cv::Mat& img) {
    cv::drawContours(img, good_contours_, -1, cv::Scalar(255, 255, 255), 2);
}

std::pair<int, int> DockDetector::getCentre(cv::Mat& img, const double& std_dev_bound) {
    centre_.first = 0;
    centre_.second = 0;
    static int freq_no_circle = 0;         // Using this variable for the purpose of finding better parameters by comparing the
                                           // number of times no circle was found.
    static int freq_non_zero_circles = 0;  // Using this variable for the purpose of finding better parameters by
                                           // comparing the number of times non zero circles were found.
    std::vector<cv::Scalar> circles;
    for (int i = 0; i < good_contours_.size(); i++) {
        cv::Scalar circle = circleDet(good_contours_[i], std_dev_bound);
        if (circle != cv::Scalar(0, 0, 0)) {
            circles.push_back(circle);
        }
    }
    if (circles.empty()) {
        freq_no_circle++;
        ROS_INFO("No circle found for %d times...Retry!!!\n", freq_no_circle);
    } else {
        freq_non_zero_circles++;
        ROS_INFO("Found %d circle(s) and found non zero circles for %d times\n", circles.size(), freq_non_zero_circles);
        for (size_t i = 0; i < circles.size(); i++) {
            cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
            int radius = cvRound(circles[i][2]);
            circle(img, center, 3, cv::Scalar(0, 255, 0), -1, 8, 0);      // drawing the center of the circle
            circle(img, center, radius, cv::Scalar(0, 0, 255), 3, 8, 0);  // drawing the outline of the circle.
        }
        double avg_x = 0, avg_y = 0, avg_r = 0;
        for (auto& x : circles) {
            avg_x += x[0];
            avg_y += x[1];
            avg_r += x[2];
        }
        avg_x /= circles.size();
        avg_y /= circles.size();
        avg_r /= circles.size();
        centre_.first = avg_x;
        centre_.second = avg_y;
    }
    return centre_;
}

cv::Mat DockDetector::getThresh() {
    return thresh_img_;
}

double DockDetector::getDistance() {
    return distance_;  // This hasn't been calculated yet..I'll do it soon.
}

void DockDetector::setHSVMin(const int& h, const int& s, const int& v) {
    hsv_min_ = cv::Scalar(h, s, v);
}

void DockDetector::setHSVMax(const int& h, const int& s, const int& v) {
    hsv_max_ = cv::Scalar(h, s, v);
}

void DockDetector::setMinArea(const int& area) {
    min_contour_area_ = area;
}

}  // namespace ariitk::dock_detection
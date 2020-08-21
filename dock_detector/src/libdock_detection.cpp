#include <dock_detector/libdock_detection.hpp>

namespace ariitk::dock_detection
{
void DockDetector::setCannyParams(const int& lower, const int& upper, const int& size)
{
  canny_param_lower_ = lower;
  canny_param_upper_ = upper;
  canny_kernel_size_ = size;
}

void DockDetector::setInterCentreDist(const double& distance)
{
  inter_centre_dist_ = distance;
}

void DockDetector::thresholdImage(cv::Mat& img)
{
  if (img.empty())
  {
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

void DockDetector::findAllContours(cv::Mat& img)
{
  cv::Canny(thresh_img_, canny_img_, canny_param_lower_, canny_param_upper_, canny_kernel_size_);

  all_contours_.clear();
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(thresh_img_, all_contours_, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
}

// cv::Scalar DockDetector::circleDet(const std::vector<cv::Point>& contour)
// {
//   Using the function used in helipad_det.
//   int n = contour.size();
//   if (n < 10)
//     return cv::Scalar(0, 0, 0);
//   cv::Point centre = 0.25 * (contour.at(0) + contour.at(round(n / 2))) +
//                      0.25 * (contour.at(round(n / 4)) + contour.at(round(3 * n / 4)));
//   double avg_r2 = 0, avg_r = 0;
//   for (int i = 0; i < n; i++)
//   {
//     avg_r += cv::norm(contour.at(i) - centre);
//     avg_r2 += cv::norm(contour.at(i) - centre) * cv::norm(contour.at(i) - centre);
//   }
//   avg_r = avg_r / n;
//   avg_r2 = avg_r2 / n;
//   double standard_deviation = std::sqrt(avg_r2 - avg_r * avg_r);
//   if (standard_deviation < 0.51 * 0.1 * avg_r)
//   {
//     return cv::Scalar(centre.x, centre.y, avg_r);
//   }
//   else
//   {
//     return cv::Scalar(0, 0, 0);
//   }
// }

// void DockDetector::drawContours(cv::Mat& img)
// {
//   cv::drawContours(img, all_contours_, -1, cv::Scalar(255, 255, 255), 2);
// }

cv::Scalar DockDetector::circleDet(cv::Mat& img)
{
  // for hough transform
  std::vector<cv::Vec3f> circles;
  cv::HoughCircles(canny_img_, circles, cv::HOUGH_GRADIENT, 78.0, 100);

  for (size_t i = 0; i < circles.size(); i++)
  {
    cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
    int radius = cvRound(circles[i][2]);
    circle(img, center, 3, cv::Scalar(0, 255, 0), -1, 8, 0);      // drawing the center of the circle
    circle(img, center, radius, cv::Scalar(0, 0, 255), 3, 8, 0);  // drawing the outline of the circle.
  }
  if (circles.empty())
  {
    ROS_INFO("No circle found...Retry!!!\n");
    return cv::Scalar(0, 0, 0);
  }
  else
  {
    ROS_INFO("Found %d circles\n", circles.size());
    int avg_x = 0, avg_y = 0, avg_r = 0;
    for (auto& x : circles)
    {
      avg_x += x[0];
      avg_y += x[1];
      avg_r += x[2];
    }
    avg_x /= circles.size();
    avg_y /= circles.size();
    avg_r /= circles.size();
    centre_.first = avg_x;
    centre_.second = avg_y;
    return cv::Scalar(centre_.first, centre_.second, avg_r);
  }
}

void DockDetector::drawContours(cv::Mat& img)
{
  cv::drawContours(img, all_contours_, -1, cv::Scalar(255, 255, 255), 2);
}

// std::pair<int, int> DockDetector::getCentre()
// {
//   Using the function used in helipad_det.
//   centre_.first = 0;
//   centre_.second = 0;
//   std::vector<cv::Scalar> circles;
//   for (int i = 0; i < all_contours_.size(); i++)
//   {
//     cv::Scalar circle = circleDet(all_contours_[i]);
//     if (circle != cv::Scalar(0, 0, 0))
//     {
//       circles.push_back(circle);
//     }
//   }
//   if (circles.empty())
//   {
//     ROS_INFO("No circle found...Retry!!!\n");
//   }
//   else if (circles.size() == 1)
//   {
//     ROS_INFO("Found one circle\n");
//     centre_.first = circles[0][0];
//     centre_.second = circles[0][1];
//   }
//   else if (circles.size() == 2)
//   {
//     ROS_INFO("Found two circles!!!\n");
//     if (circles[0][0] == circles[1][0] && abs(circles[1][0] - circles[1][1]) == inter_centre_dist_)
//     {
//       // centre_.first =
//     }
//   }
//   else if (circles.size() == 3)
//   {
//     ROS_INFO("Found three circles!!!\n");
//   }
//   else if (circles.size() == 4)
//   {
//     ROS_INFO("Found four circles!!!\n");
//   }
//   else
//   {
//     ROS_INFO("Found more than four circles!!!\n");
//   }
//   return centre_;
// }

std::pair<int, int> DockDetector::getCentre()
{
  return centre_;
}

cv::Mat DockDetector::getThresh()
{
  return thresh_img_;
}

double DockDetector::getDistance()
{
  return distance_;  // This hasn't been calculated yet..I'll do it soon.
}

void DockDetector::setHSVMin(const int& h, const int& s, const int& v)
{
  hsv_min_ = cv::Scalar(h, s, v);
}

void DockDetector::setHSVMax(const int& h, const int& s, const int& v)
{
  hsv_max_ = cv::Scalar(h, s, v);
}

}  // namespace ariitk::dock_detection
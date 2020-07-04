#pragma once

#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/imgcodecs.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <time.h>

namespace ariitk::text_detector {

class TextDetect {
  private:
    ros::Publisher surf_image_pub_;
    ros::Publisher white_text_box_pub_;
    ros::Publisher detected_box_pub_;
    ros::Publisher surf_image_match_pub_;
    ros::Publisher image_pub_preprocess_;
    ros::Subscriber image_sub_;

    cv::Mat src_;
    cv::Mat img_matches_;
    cv::Mat descriptors1_;
    cv::Mat surf_;
    cv::Mat processed_frame_;

    std::vector<cv::KeyPoint> keypoints1_;
    void imageCb(const sensor_msgs::ImageConstPtr& msg);

    // cv::Ptr<cv::xfeatures2d::SIFT> detector_ = cv::xfeatures2d::SIFT::create();
    cv::Ptr<cv::ORB> detector_ = cv::ORB::create();

    std::string workspace_path_image_;

    // cv::Ptr<cv::BFMatcher> matcher_ = cv::BFMatcher::create(cv::NORM_HAMMING);
    cv::FlannBasedMatcher matcher_ = cv::FlannBasedMatcher(cv::makePtr<cv::flann::KDTreeIndexParams>(5));
    // cv::FlannBasedMatcher(cv::makePtr<cv::flann::LshIndexParams>(20, 10, 2));

    double time1_;
    double time2_;
    int row1_;
    int row2_;
    int col1_;
    int col2_;

  public:
    void init(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
    void run();
    void findWhiteTextBox(cv::Mat& frame);
    cv::Mat preprocess(cv::Mat& img);
};

}  // namespace ariitk::text_detector

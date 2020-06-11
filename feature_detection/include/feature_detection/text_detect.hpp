#pragma once

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/imgcodecs.hpp"
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc.hpp>

namespace ariitk::text_detect{
    class text_detect {
    private:
	ros::Publisher surf_image_pub,white_text_box_pub,detected_box_pub;
    ros::Publisher surf_image_match_pub;
	ros::Publisher image_pub_preprocess;
	ros::Subscriber image_sub;
    cv::Mat src,img_matches,descriptors1, surf,processed_frame,WhiteTextBox;
    std::vector<cv::KeyPoint> keypoints1;
    cv::Ptr<cv::xfeatures2d::SURF> detector = cv::xfeatures2d::SURF::create();
    std::string workspace_path_image;
    //cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
    //cv::FlannBasedMatcher matcher = cv::FlannBasedMatcher(cv::makePtr<cv::flann::KDTreeIndexParams>(5));
    cv::BFMatcher matcher;

    public:
	void imageCb(const sensor_msgs::ImageConstPtr& msg);
	void init(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
    void feature_extract_match(cv::Mat& img);
	void run();
	cv::Mat preprocess(cv::Mat& img);
    cv::Mat findWhiteTextBox(cv::Mat& frame);
};
}
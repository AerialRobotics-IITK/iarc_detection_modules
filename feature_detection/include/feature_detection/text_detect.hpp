#pragma once

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/features2d.hpp"
//#include "opencv2/xfeatures2d.hpp"
#include "opencv2/imgcodecs.hpp"
#include <ros/ros.h>
#include <time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc.hpp>

namespace ariitk::TextDetect{
    class TextDetect {
    private:
	ros::Publisher surf_image_pub_,white_text_box_pub_,detected_box_pub_;
    ros::Publisher surf_image_match_pub_;
	ros::Publisher image_pub_preprocess_;
	ros::Subscriber image_sub_;
    cv::Mat src_,img_matches_,descriptors1_, surf_,processed_frame_;
    std::vector<cv::KeyPoint> keypoints1_;
    //cv::Ptr<cv::xfeatures2d::SIFT> detector_ = cv::xfeatures2d::SIFT::create();
    cv::Ptr<cv::ORB> detector_ = cv::ORB::create();
    std::string workspace_path_image_;
    cv::Ptr<cv::BFMatcher> matcher_=cv::BFMatcher::create(cv::NORM_HAMMING) ;
    double time1,time2;int row1,row2,col1,col2;

    public:
	void imageCb(const sensor_msgs::ImageConstPtr& msg);
	void init(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
    void feature_extract_match(cv::Mat& img);
	void run();
	cv::Mat preprocess(cv::Mat& img);
    cv::Mat findWhiteTextBox(cv::Mat& frame);
    };
}
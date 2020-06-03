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

namespace ariitk::text_detect{
    class text_detect {
    private:
	ros::Publisher surf_image_pub;
    ros::Publisher surf_image_match_pub;
	ros::Publisher image_pub_preprocess;
	ros::Subscriber image_sub;
    cv::Mat src,img_matches,descriptors1, descriptors2,processed_frame;
    std::vector<cv::KeyPoint> keypoints1;
    cv::Ptr<cv::Feature2D> detector = cv::xfeatures2d::SIFT::create(400);
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE);

    public:
	void imageCb(const sensor_msgs::ImageConstPtr& msg);
	void init(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
	void run();
	cv::Mat preprocess(cv::Mat& img);
};
}
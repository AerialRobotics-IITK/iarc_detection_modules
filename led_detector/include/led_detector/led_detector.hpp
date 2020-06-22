#pragma once

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <detector_msgs/Centre.h>
#include <led_detector/libled_detection.hpp>

namespace iarc2020::led_detection {

class LedDetectorNode {
    public:
    void init(ros::NodeHandle& nh);
    void run();

    private:
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);

    cv::Mat img_;

    ros::Subscriber img_sub_;

    ros::Publisher centre_pub_red_;
    ros::Publisher centre_pub_green_;
    ros::Publisher thresh_pub_;
    ros::Publisher contour_pub_;

    LedDetector detect_;

    detector_msgs::Centre centre_coord_red_;
    detector_msgs::Centre centre_coord_green_;
};

}  // namespace iarc2020::led_detection

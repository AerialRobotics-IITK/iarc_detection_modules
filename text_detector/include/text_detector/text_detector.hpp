#pragma once

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <detector_msgs/Centre.h>
#include <detector_msgs/Corners.h>
#include <text_detector/libtext_detection.hpp>

namespace ariitk::text_detection {

class TextDetectorNode {
  public:
    void init(ros::NodeHandle& nh);
    void run();

  private:
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);

    cv::Mat img_;

    ros::Subscriber img_sub_;

    ros::Publisher centre_pub_;
    ros::Publisher thresh_pub_;
    ros::Publisher contour_pub_;
    ros::Publisher corners_pub_;

    TextDetector detect_;

    detector_msgs::Centre centre_coord_;
    detector_msgs::Corners rect_corners_;
};

}  // namespace ariitk::text_detection

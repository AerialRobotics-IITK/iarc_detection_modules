#pragma once

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>

#include <detector_msgs/Centre.h>
#include <dock_detector/libdock_detection.hpp>

namespace ariitk::dock_detection
{
class DockDetectorNode
{
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

  DockDetector detect_;

  detector_msgs::Centre centre_coord_;
  double dock_radius_;
};

}  // namespace ariitk::dock_detection

#include <dock_detector/dock_detector.hpp>

namespace ariitk::dock_detection {
void DockDetectorNode::init(ros::NodeHandle& nh) {
    int h_min, s_min, v_min;
    int h_max, s_max, v_max;
    int canny_lower, canny_upper, canny_ker;
    int min_contour_area;
    bool verbose;

    img_sub_ = nh.subscribe("image_raw", 1, &DockDetectorNode::imageCallback, this);

    ros::NodeHandle nh_private("~");

    nh_private.getParam("h_min", h_min);
    nh_private.getParam("s_min", s_min);
    nh_private.getParam("v_min", v_min);
    nh_private.getParam("h_max", h_max);
    nh_private.getParam("s_max", s_max);
    nh_private.getParam("v_max", v_max);
    nh_private.getParam("canny_lower", canny_lower);
    nh_private.getParam("canny_upper", canny_upper);
    nh_private.getParam("canny_ker", canny_ker);
    nh_private.getParam("min_contour_area", min_contour_area);
    nh_private.getParam("std_dev_bound", std_dev_bound_);
    nh_private.getParam("verbose", verbose);

    detect_.setHSVMin(h_min, s_min, v_min);
    detect_.setHSVMax(h_max, s_max, v_max);
    detect_.setCannyParams(canny_lower, canny_upper, canny_ker);
    detect_.setMinArea(min_contour_area);
    detect_.setVerbose(verbose);

    centre_pub_ = nh_private.advertise<detector_msgs::Centre>("centre_coord", 10);
    thresh_pub_ = nh_private.advertise<sensor_msgs::Image>("thresh_img", 10);
    contour_pub_ = nh_private.advertise<sensor_msgs::Image>("contours", 10);
}

void DockDetectorNode::run() {
    if (img_.empty()) {
        return;
    }

    detect_.thresholdImage(img_);
    detect_.findGoodContours(img_);
    detect_.drawContours(img_);

    std::pair<int, int> centre_pair = detect_.getCentre(img_, std_dev_bound_);
    double distance = detect_.getDistance();
    centre_coord_.x = centre_pair.first;
    centre_coord_.y = centre_pair.second;
    centre_coord_.d = (float) distance;
    centre_coord_.header.stamp = ros::Time::now();

    sensor_msgs::ImagePtr thresh_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", detect_.getThresh()).toImageMsg();
    sensor_msgs::ImagePtr contour_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_).toImageMsg();

    thresh_pub_.publish(thresh_msg);
    contour_pub_.publish(contour_msg);
    centre_pub_.publish(centre_coord_);
}

void DockDetectorNode::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;

    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    img_ = cv_ptr->image;
}

}  // namespace ariitk::dock_detection
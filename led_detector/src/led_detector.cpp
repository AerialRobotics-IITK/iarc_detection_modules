#include <led_detector/led_detector.hpp>

namespace iarc2020::led_detection {

void LedDetectorNode::init(ros::NodeHandle& nh) {
    int h_min_red, s_min_red, v_min_red;
    int h_max_red, s_max_red, v_max_red;

    int h_min_green, s_min_green, v_min_green;
    int h_max_green, s_max_green, v_max_green;
    
    int canny_lower, canny_upper, canny_ker;
    int max_contour_area;

    img_sub_ = nh.subscribe("image_raw", 1, &LedDetectorNode::imageCallback, this);

    ros::NodeHandle nh_private("~");

    nh_private.getParam("h_min_red", h_min_red);
    nh_private.getParam("s_min_red", s_min_red);
    nh_private.getParam("v_min_red", v_min_red);
    nh_private.getParam("h_max_red", h_max_red);
    nh_private.getParam("s_max_red", s_max_red);
    nh_private.getParam("v_max_red", v_max_red);

    nh_private.getParam("h_min_green", h_min_green);
    nh_private.getParam("s_min_green", s_min_green);
    nh_private.getParam("v_min_green", v_min_green);
    nh_private.getParam("h_max_green", h_max_green);
    nh_private.getParam("s_max_green", s_max_green);
    nh_private.getParam("v_max_green", v_max_green);

    nh_private.getParam("canny_lower", canny_lower);
    nh_private.getParam("canny_upper", canny_upper);
    nh_private.getParam("canny_ker", canny_ker);
    nh_private.getParam("max_contour_area", max_contour_area);

    detect_.setHSVMinRed(h_min_red, s_min_red, v_min_red);
    detect_.setHSVMaxRed(h_max_red, s_max_red, v_max_red);
    detect_.setHSVMinGreen(h_min_green, s_min_green, v_min_green);
    detect_.setHSVMaxGreen(h_max_green, s_max_green, v_max_green);

    detect_.setCannyParams(canny_lower, canny_upper, canny_ker);
    detect_.setMaxArea(max_contour_area);

    centre_pub_red_ = nh_private.advertise<detector_msgs::Centre>("led_centre_coord_red", 10);
    centre_pub_green_ = nh_private.advertise<detector_msgs::Centre>("led_centre_coord_green", 10);
    thresh_pub_ = nh_private.advertise<sensor_msgs::Image>("led_thresh_img", 10);
    contour_pub_ = nh_private.advertise<sensor_msgs::Image>("led_contours", 10);
}

void LedDetectorNode::run() {
    if (img_.empty()) { return; };

    detect_.thresholdImage(img_);
    detect_.findGoodContours();
    detect_.drawContours(img_);

    cv::Point2f centre_pair_red = detect_.getCentreRed();
    cv::Point2f centre_pair_green = detect_.getCentreGreen();

    centre_coord_red_.x = centre_pair_red.x;
    centre_coord_red_.y = centre_pair_red.y;
    centre_coord_red_.d = -1; //CANT BE CALCULATED NEED DEPTH CAMERA
    centre_coord_red_.header.stamp = ros::Time::now();

    centre_coord_green_.x = centre_pair_green.x;
    centre_coord_green_.y = centre_pair_green.y;
    centre_coord_green_.d = -1; //CANT BE CALCULATED NEED DEPTH CAMERA
    centre_coord_green_.header.stamp = ros::Time::now();

    sensor_msgs::ImagePtr thresh_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", detect_.getThreshRed()).toImageMsg();
    sensor_msgs::ImagePtr contour_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_).toImageMsg();

    thresh_pub_.publish(thresh_msg);
    contour_pub_.publish(contour_msg);
    centre_pub_red_.publish(centre_coord_red_);
    centre_pub_green_.publish(centre_coord_green_);
}

void LedDetectorNode::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr_;

    try {
        cv_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    img_ = cv_ptr_->image;
}

}  // namespace iarc2020::led_detection

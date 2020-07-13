#include <feature_detection/text_detect.hpp>

namespace ariitk::text_detector {

cv::Mat TextDetect::preprocess(cv::Mat& img) {
    cv::Mat kernel = cv::Mat::eye(7, 7, CV_8U);
    cv::Mat result, img_hsv;
    cv::cvtColor(img, img_hsv, CV_BGR2HSV);
    cv::inRange(img_hsv, cv::Scalar(0, 0, 240), cv::Scalar(255, 15, 255), result);
    cv::morphologyEx(result, result, cv::MORPH_OPEN, kernel);
    cv::Mat element = getStructuringElement(cv::MORPH_CROSS, cv::Size(7, 7));
    cv::dilate(result, result, element);
    return result;
}

void TextDetect::init(ros::NodeHandle& nh, ros::NodeHandle& nh_private) {
    nh_private.getParam("workspace_path_image", workspace_path_image_);
    image_sub_ = nh.subscribe("image_raw", 1, &TextDetect::imageCb, this);

    image_pub_preprocess_ = nh.advertise<sensor_msgs::Image>("preprocessed_image", 1);
    white_text_box_pub_ = nh.advertise<sensor_msgs::Image>("text_box", 1);
    surf_image_match_pub_ = nh.advertise<sensor_msgs::Image>("surf_matched_image", 1);
    surf_image_pub_ = nh.advertise<sensor_msgs::Image>("surf_image", 1);
    detected_box_pub_ = nh.advertise<sensor_msgs::Image>("detected_box", 1);

    src_ = cv::imread(workspace_path_image_, cv::IMREAD_GRAYSCALE);
    if (src_.empty()) {
        ROS_ERROR("couldn't read the image");
    }

    // computing descriptors for the train image only once
    time1_ = ros::Time::now().toSec();
    detector_->detectAndCompute(src_, cv::noArray(), keypoints1_, descriptors1_);
    time2_ = ros::Time::now().toSec();
    row1_ = descriptors1_.rows;
    col1_ = descriptors1_.cols;
    ROS_INFO("Descriptor1 size: (%d,%d)", row1_, col1_);
    ROS_INFO("Time taken for input image: %lf", (time2_ - time1_));
}
void TextDetect::imageCb(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    cv::Mat frame, frame_gray;
    frame = cv_ptr->image;
    ROS_ASSERT(frame.empty() != true);
    findWhiteTextBox(frame);
}

void TextDetect::findWhiteTextBox(cv::Mat& frame) {
    cv::Mat frame_gray;
    cv::cvtColor(frame, frame_gray, CV_BGR2GRAY);
    processed_frame_ = preprocess(frame);
    std::vector<std::vector<cv::Point>> list_contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::Mat WhiteTextBox, drawing = cv::Mat::zeros(frame.size(), CV_8UC3);
    cv::findContours(processed_frame_, list_contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    std::vector<std::vector<cv::Point>> hull(list_contours.size());

    for (int i = 0; i < list_contours.size(); i++) {
        cv::convexHull(cv::Mat(list_contours[i]), hull[i]);

        if (cv::contourArea(hull[i]) < 50) {
            continue;
        }

        cv::drawContours(drawing, hull, i, cv::Scalar(255, 0, 0), 1, 8);
        detected_box_pub_.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", drawing).toImageMsg());
        cv::Rect r(cv::boundingRect(hull[i]));
        cv::Mat WhiteTextBox = frame_gray(r);

        cv::Mat text_resize;
        cv::Size size(640, 480);
        cv::resize(WhiteTextBox, text_resize, size);
        std::vector<cv::KeyPoint> keypoints2;
        cv::Mat descriptors2;
        time1_ = ros::Time::now().toSec();
        detector_->detectAndCompute(text_resize, cv::noArray(), keypoints2, descriptors2);

        row2_ = descriptors2.rows;
        col2_ = descriptors2.cols;
        ROS_INFO("Descriptor2 size: (%d,%d)", row2_, col2_);
        cv::drawKeypoints(text_resize, keypoints2, surf_);
        std::vector<std::vector<cv::DMatch>> knn_matches;

        if (col2_ != col1_) {
            break;
        }

        if (descriptors1_.empty()) {
            ROS_INFO("MatchFinder 1st descriptor empty");
            break;
        }

        if (descriptors2.empty()) {
            ROS_INFO("MatchFinder  2nd descriptor empty");
            break;
        }

        if (descriptors1_.type() != CV_32F) {
            descriptors1_.convertTo(descriptors1_, CV_32F);
        }

        if (descriptors2.type() != CV_32F) {
            descriptors2.convertTo(descriptors2, CV_32F);
        }

        if (keypoints1_.size() < 2 || keypoints2.size() < 2) {
            break;
        }

        matcher_.knnMatch(descriptors1_, descriptors2, knn_matches, 2);

        const float ratio_thresh = 0.8f;
        std::vector<cv::DMatch> good_matches;

        for (size_t i = 0; i < knn_matches.size(); i++) {
            if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance) {
                good_matches.push_back(knn_matches[i][0]);
            }
        }

        ROS_INFO("No of good matches %d", int(good_matches.size()));
        time2_ = ros::Time::now().toSec();
        ROS_INFO("Time taken for computing and matching test image: %lf", (time2_ - time1_));
        cv::drawMatches(src_, keypoints1_, text_resize, keypoints2, good_matches, img_matches_, cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<char>());

        cv::imwrite("text_box_orb_resized.jpg", surf_);
        cv::imwrite("text_box_ORB_match_resized.jpg", img_matches_);
        white_text_box_pub_.publish(cv_bridge::CvImage(std_msgs::Header(), "mono8", text_resize).toImageMsg());
        ROS_INFO("WhiteTextBox Detected");
    }
}
void TextDetect::run() {
    image_pub_preprocess_.publish(cv_bridge::CvImage(std_msgs::Header(), "mono8", processed_frame_).toImageMsg());
    surf_image_pub_.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", surf_).toImageMsg());
    surf_image_match_pub_.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_matches_).toImageMsg());
}

}  // namespace ariitk::text_detector

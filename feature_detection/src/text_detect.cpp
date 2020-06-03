#include <feature_detection/text_detect.hpp>

namespace ariitk::text_detect {
 cv::Mat text_detect::preprocess(cv::Mat& img){
     cv::cvtColor(img,img,CV_BGR2GRAY);
     return img;
 } 

 void text_detect::init(ros::NodeHandle& nh, ros::NodeHandle& nh_private) {
    //nh_private.getParam("minHessian", minHessian);
    image_sub = nh.subscribe("/firefly/camera_front/image_raw", 1, &text_detect::imageCb, this);
    image_pub_preprocess=nh.advertise<sensor_msgs::Image>("preprocessed_image", 1);
    surf_image_match_pub=nh.advertise<sensor_msgs::Image>("surf_matched_image", 1);
    surf_image_pub=nh.advertise<sensor_msgs::Image>("surf_image", 1);
    src=cv::imread("~/iarc_ws/src/IARC2020/iarc_detection_modules/feature_detection/test_image/text_iarc.png", cv::IMREAD_GRAYSCALE );
    cv::waitKey(5000);
    if(src.empty())ROS_ERROR("couldn't read the image");
    //cv::imshow("src",src);
    //cv::waitKey();
    ROS_INFO("checking");
    //computing descriptors for the train image only once
    detector->detectAndCompute( src, cv::noArray(), keypoints1, descriptors1 );
    ROS_INFO("hello");
 }
 void text_detect::imageCb(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;
	try {cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);}
    catch (cv_bridge::Exception& e) { ROS_ERROR("cv_bridge exception: %s", e.what()); }
	cv::Mat frame;
	frame = cv_ptr->image;
	ROS_ASSERT(frame.empty() != true);
    processed_frame = preprocess(frame);
    
    std::vector<cv::KeyPoint> keypoints2;
    detector->detectAndCompute( frame, cv::noArray(), keypoints2, descriptors2 );

    std::vector< cv::DMatch > matches;
    matcher->match( descriptors1, descriptors2, matches );
    cv::drawMatches( src, keypoints1, frame, keypoints2, matches, img_matches);
 }
 void text_detect::run() {
    cv_bridge::CvImage preprocessed_img;
	preprocessed_img.encoding = sensor_msgs::image_encodings::MONO8;
	preprocessed_img.header.stamp = ros::Time::now();
	preprocessed_img.image = processed_frame;
    image_pub_preprocess.publish(preprocessed_img.toImageMsg());

    cv_bridge::CvImage surf_image;
	surf_image.encoding = sensor_msgs::image_encodings::BGR8;
	surf_image.header.stamp = ros::Time::now();
	surf_image.image =  descriptors2;
    surf_image_pub.publish(surf_image.toImageMsg());

    cv_bridge::CvImage surf_image_match;
	surf_image_match.encoding = sensor_msgs::image_encodings::BGR8;
	surf_image_match.header.stamp = ros::Time::now();
	surf_image_match.image =  img_matches;
    surf_image_match_pub.publish(surf_image_match.toImageMsg());
    ROS_INFO("feature_found and matched");
 }
}// namespace ariitk::text_detect

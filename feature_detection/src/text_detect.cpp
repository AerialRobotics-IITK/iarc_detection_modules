#include <feature_detection/text_detect.hpp>

namespace ariitk::text_detect {
 cv::Mat text_detect::preprocess(cv::Mat& img){
    cv::Mat kernel = cv::Mat::eye(7, 7, CV_8U);
    cv::Mat result,img_hsv;
    cv::cvtColor(img, img_hsv, CV_BGR2HSV);
	cv::inRange(img_hsv,cv::Scalar(0,0,240),cv::Scalar(255,15,255),result);
	cv::morphologyEx(result, result, cv::MORPH_OPEN, kernel);
    cv::Mat element = getStructuringElement( cv::MORPH_CROSS,
                       cv::Size( 7, 7));
    cv::dilate(result,result, element );
	return result;
 }
 void text_detect::init(ros::NodeHandle& nh, ros::NodeHandle& nh_private) {
    nh_private.getParam("workspace_path_image", workspace_path_image);
    image_sub = nh.subscribe("/firefly/camera_front/image_raw", 1, &text_detect::imageCb, this);
    image_pub_preprocess=nh.advertise<sensor_msgs::Image>("preprocessed_image", 1);
    white_text_box_pub=nh.advertise<sensor_msgs::Image>("text_box", 1);
    surf_image_match_pub=nh.advertise<sensor_msgs::Image>("surf_matched_image", 1);
    surf_image_pub=nh.advertise<sensor_msgs::Image>("surf_image", 1);
    detected_box_pub=nh.advertise<sensor_msgs::Image>("detected_box", 1);
    src=cv::imread(workspace_path_image,
                    cv::IMREAD_GRAYSCALE );
    if(src.empty())ROS_ERROR("couldn't read the image");
    //computing descriptors for the train image only once
    detector->detectAndCompute( src, cv::noArray(), keypoints1, descriptors1 );
 }
 void text_detect::imageCb(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;
	try {cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);}
    catch (cv_bridge::Exception& e) { ROS_ERROR("cv_bridge exception: %s", e.what()); }
	cv::Mat frame,frame_gray,test_image;
	frame = cv_ptr->image;
    ROS_ASSERT(frame.empty() != true);
    test_image=findWhiteTextBox(frame);
 }

 cv::Mat text_detect::findWhiteTextBox(cv::Mat& frame){
    cv::Mat frame_gray;
    cv::cvtColor(frame,frame_gray , CV_BGR2GRAY);
    processed_frame = preprocess(frame);
    std::vector<std::vector<cv::Point>> list_contours;
	std::vector<cv::Vec4i> hierarchy;
    cv::Mat WhiteTextBox,drawing = cv::Mat::zeros(frame.size(), CV_8UC3);
    cv::findContours(processed_frame, list_contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    std::vector<std::vector<cv::Point>> hull(list_contours.size());
    cv_bridge::CvImage detected_box;
    for (int i = 0; i < list_contours.size(); i++){
        cv::convexHull(cv::Mat(list_contours[i]), hull[i]);
        if(cv::contourArea(hull[i])<50)continue;
        cv::drawContours(drawing, hull, i, cv::Scalar(255, 0, 0),1, 8);
        detected_box.encoding = sensor_msgs::image_encodings::BGR8;
        detected_box.header.stamp = ros::Time::now();
        detected_box.image = drawing;
        detected_box_pub.publish(detected_box.toImageMsg());
        cv::Rect r(cv::boundingRect(hull[i]));
        cv::Mat WhiteTextBox = frame_gray(r);

        cv::Mat text_resize;cv::Size size(640,480);
        cv::resize(WhiteTextBox,text_resize,size,cv::INTER_AREA);
        std::vector<cv::KeyPoint> keypoints2;
        cv::Mat descriptors2;

        detector->detectAndCompute(text_resize, cv::noArray(), keypoints2, descriptors2 );
        
        cv::drawKeypoints(text_resize, keypoints2, surf);

        std::vector< std::vector<cv::DMatch> > knn_matches;
        matcher.knnMatch( descriptors1, descriptors2, knn_matches, 2 );
        const float ratio_thresh = 0.8f;
        std::vector<cv::DMatch> good_matches;
        for (size_t i = 0; i < knn_matches.size(); i++)
        {
            if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
            {
                good_matches.push_back(knn_matches[i][0]);
            }
        }
        cv::drawMatches( src, keypoints1, text_resize, keypoints2, good_matches, img_matches, cv::Scalar::all(-1),
        cv::Scalar::all(-1), std::vector<char>() );

        cv::imwrite("text_box_surf_resized.jpg",surf);
        cv::waitKey(10);
        cv::imwrite("text_box_match_resized.jpg",img_matches);
        cv_bridge::CvImage white_text_box;
        white_text_box.encoding = sensor_msgs::image_encodings::MONO8;
        white_text_box.header.stamp = ros::Time::now();
        white_text_box.image = WhiteTextBox;
        white_text_box_pub.publish(white_text_box.toImageMsg());
        ROS_INFO("WhiteTextBox Detected");
        }
    return WhiteTextBox;
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
	surf_image.image =  surf;
    surf_image_pub.publish(surf_image.toImageMsg());

    cv_bridge::CvImage surf_image_match;
	surf_image_match.encoding = sensor_msgs::image_encodings::BGR8;
	surf_image_match.header.stamp = ros::Time::now();
	surf_image_match.image =  img_matches;
    surf_image_match_pub.publish(surf_image_match.toImageMsg());
    ROS_INFO("feature_found and matched");
 }
}// namespace ariitk::text_detect

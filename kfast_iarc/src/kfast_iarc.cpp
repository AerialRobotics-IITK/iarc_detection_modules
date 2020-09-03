#include <kfast_iarc/KFAST.hpp>
#include <chrono> 

namespace ariitk::kfast {

void kfast::init(ros::NodeHandle& nh, ros::NodeHandle& nh_private) {
    grayscale_image = nh.advertise<sensor_msgs::Image>("kfast_grayscale", 1);
    keypoint_vectors = nh.advertise<std_msgs::Int32MultiArray>("kfast_keypoints", 1);
    input_image = nh.subscribe("camera_front/image_raw", 1, &kfast::CallBack, this);
}

void kfast::CallBack(const sensor_msgs::ImageConstPtr& msg) {
    constexpr bool nonmax_suppress = true;
    constexpr auto warmups = 150;
    constexpr auto runs = 1000;
    constexpr auto thresh = 50;
    constexpr bool KFAST_multithread = true;

    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    cv::Mat frame, image, image_gray;
    frame = cv_ptr->image;
    cv::cvtColor(frame, image_gray, CV_BGR2GRAY);

    img_bridge.header.stamp = ros::Time::now();
    img_bridge.image = image_gray;
    std::chrono::time_point<std::chrono::high_resolution_clock> time1_,time2_;
    time1_ = std::chrono::high_resolution_clock::now();
    image =frame;
    for (int i = 0; i < warmups; ++i)
        KFAST<KFAST_multithread, nonmax_suppress>(image.data, image.cols, image.rows, static_cast<int>(image.step), KFAST_kps, thresh);
    {
        for (int32_t i = 0; i < runs; ++i) {
            KFAST<KFAST_multithread, nonmax_suppress>(image.data, image.cols, image.rows, static_cast<int>(image.step), KFAST_kps, thresh);
        }
    }
    std::vector<int> vec(3 * KFAST_kps.size(), 0);
    for (uint i = 0; i < KFAST_kps.size(); i++) {
        vec[i] = KFAST_kps[i].x;
        vec[i + 1] = KFAST_kps[i].y;
        vec[i + 2] = KFAST_kps[i].score;
    }

    kfast_keypoints.layout.dim.push_back(std_msgs::MultiArrayDimension());
    kfast_keypoints.layout.dim.push_back(std_msgs::MultiArrayDimension());
    kfast_keypoints.layout.dim[0].size = 3;
    kfast_keypoints.layout.dim[1].size = KFAST_kps.size();
    kfast_keypoints.layout.dim[0].stride = 3 * KFAST_kps.size();
    kfast_keypoints.layout.dim[1].stride = 3;
    kfast_keypoints.data = vec;
    time2_ = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed_seconds = time2_ - time1_;
    double time=elapsed_seconds.count(); 
    ROS_INFO("Time taken for computing keypoints: %lf", time);
    ROS_INFO("KFAST keypoints:%d",int(KFAST_kps.size()));
    return;
}
void kfast::run() {
    keypoint_vectors.publish(kfast_keypoints);
    grayscale_image.publish(img_bridge.toImageMsg());
}
}  // namespace ariitk::kfast

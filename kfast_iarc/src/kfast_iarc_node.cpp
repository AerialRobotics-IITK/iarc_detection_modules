#include "../include/kfast_iarc/KFAST.h"

ros::NodeHandle nh;
ros::Subscriber input_image;
ros::Publisher grayscale_image;
ros::Publisher keypoint_vectors;

std::vector<Keypoint> KFAST_kps;


void CallBack(const sensor_msgs::ImageConstPtr &msg){
  // constexpr bool display_image = true;
	constexpr bool nonmax_suppress = true;
	constexpr auto warmups = 150;
	constexpr auto runs = 1000;
	constexpr auto thresh = 50;
	constexpr bool KFAST_multithread = true;
	// constexpr char name[] = "test.jpg";
    
    int counter = 0;
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    cv::Mat frame, image, image_gray;
    frame = cv_ptr->image;
    // ROS_ASSERT(frame.empty() != true);
    cv::cvtColor(frame, image_gray, CV_BGR2GRAY);
    // if (!image.data) {
    //     std::cerr << "ERROR: failed to open image. Aborting." << std::endl;
    //     return;
    // }
    cv_bridge::CvImage img_bridge;
    img_bridge.encoding = sensor_msgs::image_encodings::MONO8;
    img_bridge.header.stamp = ros::Time::now();
    img_bridge.image = image_gray;
    grayscale_image.publish(img_bridge.toImageMsg());

    for (int i = 0; i < warmups; ++i) KFAST<KFAST_multithread, nonmax_suppress>(image.data, image.cols, image.rows, static_cast<int>(image.step), KFAST_kps, thresh);
	{
		// const high_resolution_clock::time_point start = high_resolution_clock::now();
		for (int32_t i = 0; i < runs; ++i) {
			KFAST<KFAST_multithread, nonmax_suppress>(image.data, image.cols, image.rows, static_cast<int>(image.step), KFAST_kps, thresh);
		}
		// const high_resolution_clock::time_point end = high_resolution_clock::now();
	}
    std_msgs::Int32MultiArray kfast_keypoints;
    // int arr[3] = {KFAST_kps.x, KFAST_kps.y, KFAST_kps.score};
    // int a = KFAST_kps.x;
    kfast_keypoints.layout.dim.push_back(std_msgs::MultiArrayDimension());
    kfast_keypoints.layout.dim[0].size = KFAST_kps.size();
    kfast_keypoints.layout.dim[0].stride = 1;

    kfast_keypoints.data.clear();
    kfast_keypoints.data.insert(kfast_keypoints.data.end(), KFAST_kps.begin(), KFAST_kps.end());
    keypoint_vectors.publish(KFAST_kps);

    keypoint_vectors.publish(kfast_keypoints);
    return;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "KFAST_node");
    // ros::NodeHandle nh;
    ros::Rate loopRate(10);

    input_image = nh.subscribe("/firefly/camera_front/image_raw", 1, CallBack);
    grayscale_image =  nh.advertise<sensor_msgs::Image>("kfast_grayscale", 1);    
    keypoint_vectors = nh.advertise<std_msgs::Int32MultiArray>("kfast_keypoints", 1);
    while (ros::ok()) {
        ros::spinOnce();
        loopRate.sleep();
    }
    return 0;

}
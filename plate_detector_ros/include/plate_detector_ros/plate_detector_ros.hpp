#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <plate_detector/plate_detector.hpp>
#include <util_msgs/centre.h>

namespace iarc2020::plate_detector_ros {

class PlateDetectorROS {
	public:
		PlateDetectorROS(){};
		~PlateDetectorROS(){};
		void init(ros::NodeHandle& nh);
		void run();

	private:
		void imageCallback(const sensor_msgs::ImageConstPtr& msg);
		cv::Mat img_;

		ros::Subscriber img_sub_;

		ros::Publisher centre_pub_;
		ros::Publisher thresh_pub_;
		ros::Publisher contour_pub_;

		iarc2020::plate_detector::PlateDetector detect_;

		util_msgs::centre centre_coord_;
};

} // namespace iarc2020::plate_detector_ros

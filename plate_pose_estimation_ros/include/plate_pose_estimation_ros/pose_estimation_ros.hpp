#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include <plate_pose_estimation/pose_estimation.hpp>
#include <util_msgs/centre.h>
#include <util_msgs/global_coord.h>

namespace iarc2020::plate_pose_estimation_ros {

class PlatePoseEstimationROS {
	public:
		PlatePoseEstimationROS(){};
		~PlatePoseEstimationROS(){};
		void init(ros::NodeHandle &nh);
		void run();
		void centreCallback(const util_msgs::centre &msg);
		void odomCallback(const nav_msgs::Odometry &msg);
		void odomdisplay() {
			ROS_INFO_STREAM("x: " << odom_.pose.pose.position.x
														<< "  y: " << odom_.pose.pose.position.y
														<< "  z: " << odom_.pose.pose.position.z << "\n");
		}

	private:
		util_msgs::centre centre_coord_;
		util_msgs::global_coord global_coord_;
		util_msgs::global_coord front_coord_;
		nav_msgs::Odometry odom_;
		Eigen::Vector3d glob_coord_;
		Eigen::Vector3d straight_vec_;

		ros::Subscriber centre_coord_sub_;
		ros::Subscriber odom_sub_;

		iarc2020::plate_pose_estimation::PlatePoseEstimation pose_est_;

		ros::Publisher glob_coord_pub_;
		ros::Publisher front_coord_pub_;
};

} // namespace iarc2020::plate_pose_estimation_ros

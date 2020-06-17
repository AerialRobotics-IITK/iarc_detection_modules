#pragma once

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include <detector_msgs/Centre.h>
#include <detector_msgs/GlobalCoord.h>
#include <plate_detector/libpose_estimation.hpp>

namespace iarc2020::plate_pose_estimation_ros {

class PlatePoseEstimationROS {
    public:
    void init(ros::NodeHandle& nh);
    void run();

    void centreCallback(const detector_msgs::Centre& msg) { centre_coord_ = msg; };
    void odomCallback(const nav_msgs::Odometry& msg) { odom_ = msg; };

    // TODO: verbosity flag
    void odomdisplay() {
        ROS_INFO_STREAM("x: " << odom_.pose.pose.position.x << "  y: " << odom_.pose.pose.position.y << "  z: " << odom_.pose.pose.position.z << "\n");
    }

    private:
    Eigen::Vector3d calculateGlobCoord(const double& img_x, const double& img_y, const double& dist);

    // TODO: rename messages
    detector_msgs::Centre centre_coord_;
    detector_msgs::GlobalCoord global_coord_;
    detector_msgs::GlobalCoord front_coord_;

    nav_msgs::Odometry odom_;
    Eigen::Vector3d glob_coord_;
    Eigen::Vector3d straight_vec_;

    ros::Subscriber centre_coord_sub_;
    ros::Subscriber odom_sub_;

    iarc2020::plate_pose_estimation::PlatePoseEstimation pose_est_;

    ros::Publisher glob_coord_pub_;
    ros::Publisher front_coord_pub_;
};

}  // namespace iarc2020::plate_pose_estimation_ros

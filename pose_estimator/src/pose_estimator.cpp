#include <pose_estimator/pose_estimator.hpp>

namespace iarc2020::pose_estimation {

void PoseEstimatorNode::init(ros::NodeHandle& nh, ros::NodeHandle& nh_private) {
    centre_coord_sub_ = nh.subscribe("centre_coord", 10, &PoseEstimatorNode::centreCallback, this);
    odom_sub_ = nh.subscribe("odom", 10, &PoseEstimatorNode::odomCallback, this);

    glob_coord_pub_ = nh_private.advertise<detector_msgs::GlobalCoord>("estimated_coord", 10);

    pose_est_.init();

    std::vector<double> temp_list;
    nh_private.getParam("camera_matrix", temp_list);
    pose_est_.setCamMatrix(temp_list);

    temp_list.clear();
    nh_private.getParam("cam_to_quad_rot", temp_list);
    pose_est_.setCamToQuadMatrix(temp_list);

    temp_list.clear();
    nh_private.getParam("t_cam", temp_list);
    pose_est_.setTCamMatrix(temp_list);

    bool verbose_flag = true;
    nh_private.param("verbose", verbose_flag, true);
    pose_est_.setVerbosity(verbose_flag);
}

void PoseEstimatorNode::run() {
    if ((centre_coord_.x == -1) || (centre_coord_.y == -1)) {
        glob_coord_pub_.publish(global_coord_);
        return;
    }

    glob_coord_ = calculateGlobCoord(centre_coord_.x, centre_coord_.y, centre_coord_.d);
    global_coord_.x = glob_coord_(0);
    global_coord_.y = glob_coord_(1);
    global_coord_.z = glob_coord_(2);
    glob_coord_pub_.publish(global_coord_);
}

Eigen::Vector3d PoseEstimatorNode::calculateGlobCoord(const double& img_x, const double& img_y, const double& dist) {
    pose_est_.getDistance(dist);
    pose_est_.setImgVec(img_x, img_y);
    pose_est_.CamToQuad();
    pose_est_.setQuaternion(odom_);
    pose_est_.QuadToGlob(odom_);
    return pose_est_.getGlobCoord();
}

void PoseEstimatorNode::centreCallback(const detector_msgs::Centre& msg) {
    centre_coord_ = msg;
}

void PoseEstimatorNode::odomCallback(const nav_msgs::Odometry& msg) {
    odom_ = msg;
}

}  // namespace iarc2020::pose_estimation

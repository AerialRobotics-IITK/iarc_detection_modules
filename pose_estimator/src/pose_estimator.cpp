#include <pose_estimator/pose_estimator.hpp>

namespace iarc2020::pose_estimation {

void PoseEstimatorNode::init(ros::NodeHandle& nh, ros::NodeHandle& nh_private) {
    centre_coord_sub_ = nh.subscribe("centre_coord", 10, &PoseEstimatorNode::centreCallback, this);
    odom_sub_ = nh.subscribe("odom", 10, &PoseEstimatorNode::odomCallback, this);
    corners_sub_ = nh.subscribe("corners", 10, &PoseEstimatorNode::cornersCallback, this);

    glob_coord_pub_ = nh_private.advertise<detector_msgs::GlobalCoord>("estimated_coord", 10);
    front_coord_pub_ = nh_private.advertise<detector_msgs::GlobalCoord>("front_coord", 10);
    plate_front_vec_pub_ = nh_private.advertise<detector_msgs::GlobalCoord>("plate_front_vec", 10); 

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

    nh_private.getParam("image_height", image_height_);
    nh_private.getParam("image_width", image_width_);
}

void PoseEstimatorNode::run() {
    straight_vec_ = calculateGlobCoord(image_width_ / 2, image_height_ / 2, 5);
    straight_vec_(0) -= odom_.pose.pose.position.x;  //* Converting straight_vec from position vec to direction vec
    straight_vec_(1) -= odom_.pose.pose.position.y;
    straight_vec_(2) -= odom_.pose.pose.position.z;
    front_coord_.x = straight_vec_(0);
    front_coord_.y = straight_vec_(1);
    front_coord_.z = straight_vec_(2);

    front_coord_pub_.publish(front_coord_);

    if ((centre_coord_.x == -1) || (centre_coord_.y == -1)) {
        glob_coord_pub_.publish(global_coord_);
        return;
    }

    glob_coord_ = calculateGlobCoord(centre_coord_.x, centre_coord_.y, centre_coord_.d);
    global_coord_.x = glob_coord_(0);
    global_coord_.y = glob_coord_(1);
    global_coord_.z = glob_coord_(2);
    glob_coord_pub_.publish(global_coord_);

    plate_front_vec_temp_ = calculatePlateFrontVec();
    plate_front_vec_.x = plate_front_vec_temp_(0);
    plate_front_vec_.y = plate_front_vec_temp_(1);
    plate_front_vec_.z = plate_front_vec_temp_(2);
    plate_front_vec_pub_.publish(plate_front_vec_);

    calculateCorrectionAngles();
    calculateScalingFactor();
    
}

Eigen::Vector3d PoseEstimatorNode::calculateGlobCoord(const double& img_x, const double& img_y, const double& dist) {
    pose_est_.getDistance(dist);
    pose_est_.setImgVec(img_x, img_y);
    pose_est_.CamToQuad();
    pose_est_.setQuaternion(odom_);
    pose_est_.QuadToGlob(odom_);
    return pose_est_.getGlobCoord();
}

Eigen::Vector3d PoseEstimatorNode::calculateQuadCoord(const double& img_x, const double& img_y, const double& dist) {
    pose_est_.getDistance(dist);
    pose_est_.setImgVec(img_x, img_y);
    pose_est_.CamToQuad();
    return pose_est_.getQuadCoord();
}

Eigen::Vector3d PoseEstimatorNode::calculateQuadCoord2(const double& img_x, const double& img_y, const double& dist) {
    pose_est_.getDistance(dist);
    pose_est_.setImgVec(img_x, img_y);
    pose_est_.CamToQuad2();
    return pose_est_.getQuadCoord();
}

Eigen::Vector3d PoseEstimatorNode::calculatePlateFrontVec() {
    c1_quad_coord_ = calculateQuadCoord(corners_.c1_x, corners_.c1_y, centre_coord_.d);
    c2_quad_coord_ = calculateQuadCoord(corners_.c2_x, corners_.c2_y, centre_coord_.d);
    c3_quad_coord_ = calculateQuadCoord(corners_.c3_x, corners_.c3_y, centre_coord_.d);
    c4_quad_coord_ = calculateQuadCoord(corners_.c4_x, corners_.c4_y, centre_coord_.d);

    Eigen::Vector3d cross_p;
    cross_p = (c2_quad_coord_ - c4_quad_coord_).cross(c1_quad_coord_ - c3_quad_coord_);
    pose_est_.setQuaternion(odom_);
    pose_est_.QuadToGlobPlateFrontVec(odom_, cross_p);
    return pose_est_.getPlateFrontVec();
}

void PoseEstimatorNode::calculateCorrectionAngles() {
    //* Calculating Correction Angles(to be shifted to traj pkg after completion)
    float ang_x = 0, ang_y = 0, ang_z = 0, mod_v1 = 0, mod_v2 = 0;
    Eigen::Vector3d temp, v1, v2;

    v1 = Eigen::Vector3d(0, straight_vec_(1), straight_vec_(2));
    v2 = Eigen::Vector3d(0, plate_front_vec_temp_(1), plate_front_vec_temp_(2));
    mod_v1 = sqrt(v1.dot(v1));
    mod_v2 = sqrt(v2.dot(v2));
    ang_x = acos(v1.dot(v2)/(mod_v1*mod_v2)) - M_PI;

    v1 = Eigen::Vector3d(straight_vec_(0), 0, straight_vec_(2));
    v2 = Eigen::Vector3d(plate_front_vec_temp_(0), 0, plate_front_vec_temp_(2));
    mod_v1 = sqrt(v1.dot(v1));
    mod_v2 = sqrt(v2.dot(v2));
    ang_y = acos(v1.dot(v2)/(mod_v1*mod_v2)) - M_PI;

    v1 = Eigen::Vector3d(straight_vec_(0), straight_vec_(1), 0);
    v2 = Eigen::Vector3d(plate_front_vec_temp_(0), plate_front_vec_temp_(1), 0);
    mod_v1 = sqrt(v1.dot(v1));
    mod_v2 = sqrt(v2.dot(v2));
    ang_z = acos(v1.dot(v2)/(mod_v1*mod_v2)) - M_PI;

    std::cout << "[ " << ang_x << " " << ang_y << " " << ang_z << " ]" << std::endl << std::endl;
}

void PoseEstimatorNode::calculateScalingFactor() {
    c1_quad_coord_ = calculateQuadCoord2(corners_.c1_x, corners_.c1_y, centre_coord_.d);
    c2_quad_coord_ = calculateQuadCoord2(corners_.c2_x, corners_.c2_y, centre_coord_.d);
    c3_quad_coord_ = calculateQuadCoord2(corners_.c3_x, corners_.c3_y, centre_coord_.d);
    c4_quad_coord_ = calculateQuadCoord2(corners_.c4_x, corners_.c4_y, centre_coord_.d);

    Eigen::Vector3d cross_p;
    cross_p = (c1_quad_coord_ - c2_quad_coord_).cross(c3_quad_coord_ - c2_quad_coord_);
    area_ = sqrt(cross_p.dot(cross_p));
    dist_ = actual_area_/area_;
    std::cout << "[ " << dist_ << " ]" << std::endl << std::endl;
}

void PoseEstimatorNode::centreCallback(const detector_msgs::Centre& msg) {
    centre_coord_ = msg;
}

void PoseEstimatorNode::odomCallback(const nav_msgs::Odometry& msg) {
    odom_ = msg;
}

void PoseEstimatorNode::cornersCallback(const detector_msgs::Corners& msg) {
    corners_ = msg;
}

}  // namespace iarc2020::pose_estimation

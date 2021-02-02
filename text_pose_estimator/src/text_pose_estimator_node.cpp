#include <text_pose_estimator/text_pose_estimator.hpp>

using namespace ariitk::text_pose_estimation;

int main(int argc, char** argv) {
    ros::init(argc, argv, "pose_estimator_node");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    TextPoseEstimatorNode pose_est;

    pose_est.init(nh, nh_private);

    ros::Rate loop_rate(10);

    while (ros::ok()) {
        loop_rate.sleep();
        ros::spinOnce();
        pose_est.run();
    }

    return 0;
}

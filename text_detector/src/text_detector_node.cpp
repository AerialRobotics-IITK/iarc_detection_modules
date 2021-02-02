#include <text_detector/text_detector.hpp>

using namespace ariitk::text_detection;

int main(int argc, char** argv) {
    ros::init(argc, argv, "text_detector_node");
    ros::NodeHandle nh;

    TextDetectorNode detect;

    detect.init(nh);

    ros::Rate loop_rate(20);

    while (ros::ok()) {
        detect.run();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

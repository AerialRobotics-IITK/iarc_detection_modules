#include <led_detector/led_detector.hpp>

using namespace ariitk::led_detection;

int main(int argc, char** argv) {
    ros::init(argc, argv, "led_detector_node");
    ros::NodeHandle nh;

    LedDetectorNode detect;

    detect.init(nh);

    ros::Rate loop_rate(20);

    while (ros::ok()) {
        detect.run();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

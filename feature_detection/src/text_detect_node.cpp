#include <feature_detection/text_detect.hpp>

using namespace ariitk::text_detect;

int main(int argc,char** argv)
{
    ros::init(argc,argv,"text_detect_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    
    text_detect detect;

    detect.init(nh,nh_private);

    ros::Rate loopRate(2);

    while(ros::ok())
    {
        ros::spinOnce();
        detect.run();
        loopRate.sleep();
    }
    return 0;
}

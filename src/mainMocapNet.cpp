#include <lipm_control/controlMocapNet.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lipm_control_mocapnet_node");
    ros::NodeHandle nh;
    controlMocapNet* lc;
    lc = new controlMocapNet(nh);
    lc->run();
    ros::spin();
    delete lc;
    return 0;
}
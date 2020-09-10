#include <lipm_control/control.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lipm_control_node");
    ros::NodeHandle nh;
    control* lc;
    lc = new control(nh);
    lc->run();
    ros::spin();
    delete lc;
    return 0;
}
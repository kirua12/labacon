#include <stanley.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "stanley_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    Stanley stanley(nh,pnh);
    ros::spin();
    return 0;
}


#include "ginger_chassis/MobileBase.h"

using namespace Ginger;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "base_control_node");

    MobileBase m_base(0.234, 0.08);
    m_base.baseInitialize();

    ros::Rate loop_rate = ros::Rate(100);
    while(ros::ok()){
        ros::spinOnce();
	m_base.getOdom();
	loop_rate.sleep();
    }

}

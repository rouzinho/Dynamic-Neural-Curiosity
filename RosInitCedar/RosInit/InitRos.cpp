#include "InitRos.h"
#include "ros/ros.h"


//#include <sstream>


/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
InitRos::InitRos(){}


void InitRos::run()
{
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "cedar");
    //ros::Rate loop_rate(20);
    //ros::Rate loop_rate(80);
    //loop_rate.sleep();
}
void requestStop()
{
    ros::shutdown();
}

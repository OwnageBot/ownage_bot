#include <stdio.h>

#include <ros/ros.h>
#include "object_picker.h"

using namespace std;

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "action_provider");
    ros::NodeHandle _n("action_provider");

    bool use_robot;
    _n.param<bool>("use_robot", use_robot, true);
    printf("\n");
    ROS_INFO("use_robot flag set to %s", use_robot==true?"true":"false");

    printf("\n");
    ObjectPicker left_ctrl("action_provider","left", !use_robot);
    ROS_INFO("READY! Waiting for service messages..\n");

    ros::spin();
    return 0;
}


#include <ros/ros.h>
#include "../include/CameraConeDetection.h"
#include <sgtdv_msgs/ConeArr.h>
#include <std_msgs/Empty.h>

int main(int argc, char** argv)
{
    CameraConeDetection cameraConeDetection;

    ros::init(argc, argv, "cameraConeDetection");
    ros::NodeHandle handle;

    ros::Publisher conePublisher = handle.advertise<sgtdv_msgs::ConeArr>("camera_cones", 1);
    ros::Publisher signalPublisher = handle.advertise<std_msgs::Empty>("camera_ready", 1);

    cameraConeDetection.SetConePublisher(conePublisher);
    cameraConeDetection.SetSignalPublisher(signalPublisher);

    cameraConeDetection.Do();

    return 0;
}


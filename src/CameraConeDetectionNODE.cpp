/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský
/*****************************************************/

#include <ros/ros.h>
#include "../include/CameraConeDetection.h"

int main(int argc, char** argv)
{
    CameraConeDetection cameraConeDetection;

    ros::init(argc, argv, "cameraConeDetection");
    ros::NodeHandle handle;

    ros::Publisher conePublisher = handle.advertise<sgtdv_msgs::ConeArr>("camera_cones", 1);
    ros::Publisher signalPublisher = handle.advertise<std_msgs::Empty>("camera_ready", 1);

    ros::Publisher carStatePublisher = handle.advertise<sgtdv_msgs::CarState>("camera_pose", 1);
    cameraConeDetection.SetcarStatePublisher(carStatePublisher);

    cameraConeDetection.SetConePublisher(conePublisher);
    cameraConeDetection.SetSignalPublisher(signalPublisher);

    cameraConeDetection.Do();

    return 0;
}


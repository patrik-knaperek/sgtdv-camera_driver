/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský
/*****************************************************/

#include <ros/ros.h>
#include "../include/CameraConeDetection.h"

int main(int argc, char **argv) {
    CameraConeDetection cameraConeDetection;

    ros::init(argc, argv, "cameraConeDetection");
    ros::NodeHandle handle;

    ros::Publisher conePublisher = handle.advertise<sgtdv_msgs::ConeArr>("camera_cones", 1);
    ros::Publisher signalPublisher = handle.advertise<std_msgs::Empty>("camera_ready", 1);
#ifdef CAMERA_DETECTION_FAKE_LIDAR
    ros::Publisher lidarConePublisher = handle.advertise<sgtdv_msgs::Point2DArr>("lidar_cones", 1);
#endif//CAMERA_DETECTION_FAKE_LIDAR

#ifdef CAMERA_DETECTION_CARSTATE
    ros::Publisher carStatePublisher = handle.advertise<sgtdv_msgs::CarState>("camera_pose", 1);
    cameraConeDetection.SetCarStatePublisher(carStatePublisher);
#endif//CAMERA_DETECTION_CARSTATE

    cameraConeDetection.SetConePublisher(conePublisher);
#ifdef CAMERA_DETECTION_FAKE_LIDAR
    cameraConeDetection.SetLidarConePublisher(lidarConePublisher);
#endif//CAMERA_DETECTION_FAKE_LIDAR
    cameraConeDetection.SetSignalPublisher(signalPublisher);

    cameraConeDetection.Do();

    return 0;
}


/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský
/*****************************************************/

#include <ros/ros.h>
#include "../include/camera_cone_detection.h"
#include <sgtdv_msgs/DebugState.h>
#include "../../SGT_Macros.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "camera_cone_detection");
  ros::NodeHandle handle;

  CameraConeDetection camera_cone_detection(handle);

  camera_cone_detection.update();

  return 0;
}


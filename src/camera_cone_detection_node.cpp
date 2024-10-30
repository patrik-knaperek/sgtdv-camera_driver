/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský
/*****************************************************/

#include "camera_cone_detection.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "camera_cone_detection");
  ros::NodeHandle handle;

  CameraConeDetection camera_cone_detection(handle, CameraConeDetection::loadParams(handle));

  ros::Rate loop_rate(FPS);
  while (ros::ok())
  {
    camera_cone_detection.update();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}


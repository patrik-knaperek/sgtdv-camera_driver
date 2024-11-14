/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský, Patrik Knaperek
/*****************************************************/

#include "camera_driver.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "camera_driver");
  ros::NodeHandle handle;

  CameraDriver camera_driver(handle, CameraDriver::loadParams(handle));

  ros::Rate loop_rate(FPS);
  while (ros::ok())
  {
    camera_driver.update();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}


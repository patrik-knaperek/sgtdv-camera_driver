/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský
/*****************************************************/

#include <ros/ros.h>
#include <ros/package.h>
#include "../include/camera_cone_detection.h"
#include <sgtdv_msgs/DebugState.h>
#include "../../SGT_Macros.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "camera_cone_detection");
  ros::NodeHandle handle;

  CameraConeDetection camera_cone_detection;

  std::string path_to_package = ros::package::getPath("camera_cone_detection");	

  std::string obj_names_filename;	
  handle.getParam("/obj_names_filename", obj_names_filename);

  std::string cfg_filename;
  handle.getParam("/cfg_filename", cfg_filename);

  std::string weights_filename;
  handle.getParam("/weights_filename", weights_filename);

  std::string out_video_filename;
  handle.getParam("/output_video_filename", out_video_filename);

  std::string out_svo_filename;
  handle.getParam("/output_svo_filename", out_svo_filename);

    std::string in_stream;
  handle.getParam("/input_stream", in_stream);
  camera_cone_detection.setFilenames(path_to_package + obj_names_filename,
                  path_to_package + cfg_filename,
                  path_to_package + weights_filename,
                  path_to_package + out_video_filename,
                  path_to_package + out_svo_filename,
                  path_to_package + in_stream);

  bool publish_carstate, camera_show, fake_lidar, console_show, record_video, record_video_svo;
  handle.getParam("/publish_carstate", publish_carstate);
  handle.getParam("/camera_show", camera_show);
  handle.getParam("/fake_lidar", fake_lidar);
  handle.getParam("/console_show", console_show);
  handle.getParam("/record_video", record_video);
  handle.getParam("/record_video_svo", record_video_svo);
  camera_cone_detection.setMacros(
    publish_carstate, camera_show, fake_lidar, console_show, record_video, record_video_svo);

  camera_cone_detection.setConePublisher(handle.advertise<sgtdv_msgs::ConeStampedArr>("camera_cones", 1));
  camera_cone_detection.setSignalPublisher(handle.advertise<std_msgs::Empty>("camera_ready", 1));
  
  if(fake_lidar) 
    camera_cone_detection.setLidarConePublisher(handle.advertise<sgtdv_msgs::Point2DStampedArr>("lidar_cones", 1));

  if(publish_carstate)
  {
    camera_cone_detection.setCarStatePublisher(
      handle.advertise<geometry_msgs::PoseWithCovarianceStamped>("camera_pose", 1));
    ros::Subscriber resetOdomSubscriber 
      = handle.subscribe("reset_odometry", 1, &CameraConeDetection::resetOdomCallback, &camera_cone_detection);
  }

#ifdef SGT_DEBUG_STATE
  ros::Publisher vis_debug_pub 
    = handle.advertise<sgtdv_msgs::DebugState>("camera_cone_detection_debug_state", 1);
  camera_cone_detection.setVisDebugPublisher(vis_debug_pub);
#endif

  camera_cone_detection.update();

  return 0;
}


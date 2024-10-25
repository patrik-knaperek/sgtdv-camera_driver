/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský
/*****************************************************/

#include <ros/ros.h>
#include <ros/package.h>
#include "../include/CameraConeDetection.h"
#include <sgtdv_msgs/DebugState.h>
#include "../../SGT_Macros.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "cameraConeDetection");
    ros::NodeHandle handle;

    CameraConeDetection cameraConeDetection;

	std::string pathToPackage = ros::package::getPath("camera_cone_detection");	

	std::string objNamesFilename;	
	handle.getParam("/obj_names_filename", objNamesFilename);
	
	std::string cfgFilename;
	handle.getParam("/cfg_filename", cfgFilename);
	
	std::string weightsFilename;
	handle.getParam("/weights_filename", weightsFilename);

	std::string outVideoFilename;
	handle.getParam("/output_video_filename", outVideoFilename);

    std::string outSvoFilename;
	handle.getParam("/output_svo_filename", outSvoFilename);

    std::string inSvoFilename;
	handle.getParam("/input_svo_filename", inSvoFilename);
	cameraConeDetection.SetFilenames(pathToPackage + objNamesFilename,
									pathToPackage + cfgFilename,
									pathToPackage + weightsFilename,
									pathToPackage + outVideoFilename,
                                    pathToPackage + outSvoFilename,
                                    pathToPackage + inSvoFilename);

    bool publish_carstate, camera_show, fake_lidar, console_show, record_video, record_video_svo;
    handle.getParam("macros/publish_carstate", publish_carstate);
    handle.getParam("macros/camera_show", camera_show);
    handle.getParam("macros/fake_lidar", fake_lidar);
    handle.getParam("macros/console_show", console_show);
    handle.getParam("macros/record_video", record_video);
    handle.getParam("macros/record_video_svo", record_video_svo);
    cameraConeDetection.SetMacros(publish_carstate, camera_show, fake_lidar, console_show, record_video, record_video_svo);

    cameraConeDetection.SetConePublisher(handle.advertise<sgtdv_msgs::ConeStampedArr>("camera_cones", 1));
    cameraConeDetection.SetSignalPublisher(handle.advertise<std_msgs::Empty>("camera_ready", 1));
    
    if(fake_lidar) cameraConeDetection.SetLidarConePublisher(handle.advertise<sgtdv_msgs::Point2DStampedArr>("lidar_cones", 1));

    if(publish_carstate)
    {
        cameraConeDetection.SetCarStatePublisher(handle.advertise<geometry_msgs::PoseWithCovarianceStamped>("camera_pose", 1));
        ros::Subscriber resetOdomSubscriber = handle.subscribe("reset_odometry", 1, &CameraConeDetection::ResetOdomCallback, &cameraConeDetection);
    }

#ifdef SGT_DEBUG_STATE
    ros::Publisher cameraConeDetectionDebugStatePublisher = handle.advertise<sgtdv_msgs::DebugState>("camera_cone_detection_debug_state", 1);
    cameraConeDetection.SetVisDebugPublisher(cameraConeDetectionDebugStatePublisher);
#endif

    cameraConeDetection.Do();

    return 0;
}


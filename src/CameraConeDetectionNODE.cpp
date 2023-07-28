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
    CameraConeDetection cameraConeDetection;

    ros::init(argc, argv, "cameraConeDetection");
    ros::NodeHandle handle;

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
	cameraConeDetection.SetFilenames(pathToPackage + objNamesFilename,
									pathToPackage + cfgFilename,
									pathToPackage + weightsFilename,
									pathToPackage + outVideoFilename,
                                    pathToPackage + outSvoFilename);

    ros::Publisher conePublisher = handle.advertise<sgtdv_msgs::ConeStampedArr>("camera_cones", 1);
    ros::Publisher signalPublisher = handle.advertise<std_msgs::Empty>("camera_ready", 1);
    
#ifdef CAMERA_DETECTION_FAKE_LIDAR
    ros::Publisher lidarConePublisher = handle.advertise<sgtdv_msgs::Point2DStampedArr>("lidar_cones", 1);
#endif//CAMERA_DETECTION_FAKE_LIDAR

#ifdef CAMERA_DETECTION_CARSTATE
    ros::Publisher carStatePublisher = handle.advertise<geometry_msgs::PoseWithCovarianceStamped>("camera_pose", 1);
    cameraConeDetection.SetCarStatePublisher(carStatePublisher);
#endif//CAMERA_DETECTION_CARSTATE

    cameraConeDetection.SetConePublisher(conePublisher);
#ifdef CAMERA_DETECTION_FAKE_LIDAR
    cameraConeDetection.SetLidarConePublisher(lidarConePublisher);
#endif//CAMERA_DETECTION_FAKE_LIDAR
    cameraConeDetection.SetSignalPublisher(signalPublisher);

#ifdef SGT_DEBUG_STATE
    ros::Publisher cameraConeDetectionDebugStatePublisher = handle.advertise<sgtdv_msgs::DebugState>("camera_cone_detection_debug_state", 1);
    cameraConeDetection.SetVisDebugPublisher(cameraConeDetectionDebugStatePublisher);
#endif

    cameraConeDetection.Do();

    return 0;
}


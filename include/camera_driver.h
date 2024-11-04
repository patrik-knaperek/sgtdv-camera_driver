/*****************************************************/
//Organization: Stuba Green Team
//Authors: Matúš Tomšík, Juraj Krasňanský, Patrik Knaperek
/*****************************************************/

/* ROS */
#include <ros/ros.h>
#include <ros/package.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

/* SGT-DV */
#include <sgtdv_msgs/ConeStampedArr.h>
#include <sgtdv_msgs/DebugState.h>
#include "SGT_Macros.h"
#include "SGT_Utils.h"
#include "camera_cone_detection.h"

/* Darknet */
#include "yolo_v2_class.hpp"    // imported functions from DLL

/* ZED, OpenCV */
#define ZED_STEREO
#define OPENCV

#ifdef ZED_STEREO

#include <sl/Camera.hpp>

#pragma comment(lib, "sl_core64.lib")
#pragma comment(lib, "sl_input64.lib")
#pragma comment(lib, "sl_zed64.lib")
#endif  // ZED_STEREO

#include <opencv2/opencv.hpp>            // C++
#include <opencv2/core/version.hpp>
#include <opencv2/highgui/highgui_c.h>

#ifndef CV_VERSION_EPOCH     // OpenCV 3.x and 4.x

#include <opencv2/videoio/videoio.hpp>

#define OPENCV_VERSION CVAUX_STR(CV_VERSION_MAJOR)"" CVAUX_STR(CV_VERSION_MINOR)"" CVAUX_STR(CV_VERSION_REVISION)
#ifndef USE_CMAKE_LIBS
#pragma comment(lib, "opencv_world" OPENCV_VERSION ".lib")
#ifdef TRACK_OPTFLOW
#pragma comment(lib, "opencv_cudaoptflow" OPENCV_VERSION ".lib")
#pragma comment(lib, "opencv_cudaimgproc" OPENCV_VERSION ".lib")
#pragma comment(lib, "opencv_core" OPENCV_VERSION ".lib")
#pragma comment(lib, "opencv_imgproc" OPENCV_VERSION ".lib")
#pragma comment(lib, "opencv_highgui" OPENCV_VERSION ".lib")
#endif    // TRACK_OPTFLOW
#endif    // USE_CMAKE_LIBS
#else     // OpenCV 2.x
#define OPENCV_VERSION CVAUX_STR(CV_VERSION_EPOCH)"" CVAUX_STR(CV_VERSION_MAJOR)"" CVAUX_STR(CV_VERSION_MINOR)
#ifndef USE_CMAKE_LIBS
#pragma comment(lib, "opencv_core" OPENCV_VERSION ".lib")
#pragma comment(lib, "opencv_imgproc" OPENCV_VERSION ".lib")
#pragma comment(lib, "opencv_highgui" OPENCV_VERSION ".lib")
#pragma comment(lib, "opencv_video" OPENCV_VERSION ".lib")
#endif    // USE_CMAKE_LIBS
#endif    // CV_VERSION_EPOCH

constexpr int FPS = 30;

class CameraDriver
{
public:
  struct Params
  {
    std::string names_file;
    std::string out_video_file;
    std::string out_svo_file;
    std::string in_svo_file;
    bool publish_carstate;
    bool camera_show;
    bool console_show;
    bool record_video;
    bool record_video_svo;
  };

  static CameraConeDetection::Params loadNNParams(const ros::NodeHandle& nh)
  {
    const auto path_to_package = ros::package::getPath("camera_driver");
    std::string filename_temp;
    CameraConeDetection::Params params;

    Utils::loadParam(nh, "/cfg_filename", &filename_temp);
    params.cfg_file = path_to_package + filename_temp;
    
    Utils::loadParam(nh, "/weights_filename", &filename_temp);
    params.weights_file = path_to_package + filename_temp;

    return params;
  }

public:
  CameraDriver(ros::NodeHandle& nh, const CameraConeDetection::Params& nn_params);

  ~CameraDriver();

  void update();

private:

  void loadParams(const ros::NodeHandle& nh);

  void initialize(void);
  
  bool resetOdomCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  
  CameraConeDetection camera_cone_detection_;
  
  sl::Camera zed_; // ZED-camera object
  sl::MODEL cam_model_;
  sl::Pose camera_pose_;
  sl::SensorsData sensors_data_;

  cv::VideoWriter output_video_;

  Params params_;

  ros::Publisher cone_pub_;
  ros::Publisher carstate_pub_;
  ros::ServiceServer reset_odom_server_;

#ifdef SGT_DEBUG_STATE
  ros::Publisher vis_debug_pub_;
  size_t num_of_detected_cones_ = 0;
#endif

  cv::Mat slMat2cvMat(const sl::Mat &input) const;

  cv::Mat zedCaptureRGB(void);

  cv::Mat zedCapture3D(void);
};

/*****************************************************/
//Organization: Stuba Green Team
//Authors: Matúš Tomšík, Juraj Krasňanský
/*****************************************************/

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <sgtdv_msgs/ConeStampedArr.h>
#include <sgtdv_msgs/ConeStamped.h>
#include <sgtdv_msgs/Point2DStamped.h>
#include <sgtdv_msgs/Point2DStampedArr.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sgtdv_msgs/DebugState.h>
#include "../../SGT_Macros.h"
#include "../../SGT_Utils.h"
#include <chrono>


#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <queue>
#include <fstream>
#include <thread>
#include <future>
#include <atomic>
#include <mutex>         // std::mutex, std::unique_lock
#include <cmath>
#include <stdio.h>


#include "yolo_v2_class.hpp"    // imported functions from DLL

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

constexpr int FPS = 20;

class CameraConeDetection
{
public:
  struct Params
  {
    std::string names_file;
    std::string cfg_file;
    std::string weights_file;
    std::string out_video_file;
    std::string out_svo_file;
    std::string in_svo_file;
    bool publish_carstate;
    bool camera_show;
    bool console_show;
    bool record_video;
    bool record_video_svo;
  };

  static Params loadParams(const ros::NodeHandle& nh)
  {
    const auto path_to_package = ros::package::getPath("camera_cone_detection");
    std::string filename_temp;
    Params params;
    
    Utils::loadParam(nh, "/obj_names_filename", &filename_temp);
    params.names_file = path_to_package + filename_temp;
    
    Utils::loadParam(nh, "/cfg_filename", &filename_temp);
    params.cfg_file = path_to_package + filename_temp;
    
    Utils::loadParam(nh, "/weights_filename", &filename_temp);
    params.weights_file = path_to_package + filename_temp;
    
    Utils::loadParam(nh, "/output_video_filename", &filename_temp);
    params.out_video_file = path_to_package + filename_temp;
    
    Utils::loadParam(nh, "/output_svo_filename", &filename_temp);
    params.out_svo_file = path_to_package + filename_temp;
    
    Utils::loadParam(nh, "/input_stream", &filename_temp);
    params.in_svo_file = path_to_package + filename_temp;

    Utils::loadParam(nh, "/publish_carstate", false, &params.publish_carstate);
    Utils::loadParam(nh, "/camera_show", false, &params.camera_show);
    Utils::loadParam(nh, "/console_show", false, &params.console_show);
    Utils::loadParam(nh, "/record_video", false, &params.record_video);
    Utils::loadParam(nh, "/record_video_svo", false, &params.record_video_svo);

  return params;
  };

public:
  explicit CameraConeDetection(ros::NodeHandle& nh, const Params& params);

  ~CameraConeDetection();

  void update();

private:

  void initialize(void);
  
  void resetOdomCallback(const std_msgs::Empty::ConstPtr& msg);
  
  enum CONE_CLASSES
  {
    YELLOW,
    BLUE,
    ORANGE_SMALL,
    ORANGE_BIG
  };

  static constexpr float DETECT_TH = 0.2;
  
  Detector detector_; // Darknet NN object
  std::vector<std::string> obj_names_;
  
  sl::Camera zed_; // ZED-camera object
  sl::MODEL cam_model_;
  sl::Pose camera_pose_;
  sl::SensorsData sensors_data_;

  cv::VideoWriter output_video_;

  Params params_;

  ros::Publisher cone_pub_;
  ros::Publisher carstate_pub_;
  ros::Subscriber reset_odom_sub_;

#ifdef SGT_DEBUG_STATE
  ros::Publisher vis_debug_pub_;
  size_t num_of_detected_cones_ = 0;
#endif

  float getMedian(std::vector<float> &v) const;

  void get3dCoordinates(std::vector <bbox_t>* bbox_vect, const cv::Mat& xyzrgba) const;

  cv::Mat slMat2cvMat(const sl::Mat &input) const;

  cv::Mat zedCaptureRGB(void);

  cv::Mat zedCapture3D(void);

  void showConsoleResult(const std::vector <bbox_t>& result_vec) const;

  void drawBoxes(cv::Mat* mat_img, const std::vector <bbox_t>& result_vec,
                const int current_det_fps, const int current_cap_fps) const;
  
  void getObjectsNamesFromFile(void);
};

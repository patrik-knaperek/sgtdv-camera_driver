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
constexpr int TIME_PER_FRAME = 1000 / FPS;

class CameraConeDetection
{
public:
  explicit CameraConeDetection(ros::NodeHandle& nh);

  ~CameraConeDetection();

  void update();

  void predict(Detector &detector, sl::MODEL &cam_model);

  enum CONE_CLASSES
  {
    YELLOW,
    BLUE,
    ORANGE_SMALL,
    ORANGE_BIG
  };

private:
  void loadParams(const ros::NodeHandle& nh);
  void resetOdomCallback(const std_msgs::Empty::ConstPtr& msg);

  struct Params
  {
    // std::string names_file;
    std::string cfg_file;
    std::string weights_file;
    std::string out_video_file;
    std::string out_svo_file;
    std::string in_svo_file;
    bool publish_carstate;
    bool camera_show;
    bool fake_lidar;
    bool console_show;
    bool record_video;
    bool record_video_svo;
  };
  
  sl::Camera zed_; // ZED-camera object

  cv::VideoWriter output_video_;
  float const thresh = 0.2;

  ros::Publisher signal_pub_;
  ros::Publisher cone_pub_;
  ros::Publisher lidar_cone_pub_;
  ros::Publisher carstate_pub_;
  ros::Subscriber reset_odom_sub_;

  Params params_;

#ifdef SGT_DEBUG_STATE
  ros::Publisher vis_debug_pub_;
  size_t num_of_detected_cones_ = 0;
#endif

  float getMedian(std::vector<float> &v);

  std::vector <bbox_t> get_3d_coordinates(std::vector <bbox_t> bbox_vect, cv::Mat xyzrgba);

  cv::Mat slMat2cvMat(sl::Mat &input);

  cv::Mat zedCaptureRGB(sl::Camera &zed);

  cv::Mat zedCapture3D(sl::Camera &zed);

  void
  showConsoleResult(std::vector <bbox_t> const result_vec, std::vector <std::string> const obj_names, int frame_id);

  cv::Mat drawBoxes(cv::Mat mat_img, std::vector <bbox_t> result_vec, std::vector <std::string> obj_names,
                      int current_det_fps, int current_cap_fps);
  // std::vector<std::string> objects_names_from_file(std::string const input_stream);
};

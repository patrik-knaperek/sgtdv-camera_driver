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
  CameraConeDetection();

  ~CameraConeDetection();

  void setSignalPublisher(ros::Publisher signal_publisher);
  void setConePublisher(ros::Publisher main_publisher);
  void setFilenames(std::string names, std::string cfg, std::string weights, 
                    std::string out, std::string out_svo, std::string in_stream)
  {
    names_file_ = names;
    cfg_file_ = cfg;
    weights_file_ = weights;
    out_videofile_ = out;
    out_svofile_ = out_svo;
    input_stream_ = in_stream;
  };

  void setMacros(const bool carstate, const bool camera_show, const bool fake_lidar, const bool console_show, 
                const bool record_video, const bool record_video_svo)
  {
    publish_carstate_ = carstate;
    camera_show_ = camera_show;
    fake_lidar_ = fake_lidar;
    console_show_ = console_show;
    record_video_ = record_video;
    record_video_svo_ = record_video;
  };

#ifdef SGT_DEBUG_STATE
  void setVisDebugPublisher(ros::Publisher vis_debug_pub) { vis_debug_pub_ = vis_debug_pub; }
#endif

  void setLidarConePublisher(ros::Publisher lidar_cone_pub);

  void setCarStatePublisher(ros::Publisher carstate_pub);
  void resetOdomCallback(const std_msgs::Empty::ConstPtr& msg);


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
  std::string names_file_, cfg_file_, weights_file_, out_videofile_, out_svofile_;
  bool publish_carstate_ = false, camera_show_ = false, fake_lidar_ = false, console_show_ = false,
      record_video_ = false, record_video_svo_ = false;

  //std::string input_stream_ = ros::package::getPath("camera_cone_detection") + "/Darknet_cone_detection/druha_jazda.svo";
  std::string input_stream_ = "zed_camera";
  sl::Camera zed_; // ZED-camera

  cv::VideoWriter output_video_;
  float const thresh = 0.2;

  ros::Publisher signal_pub_;
  ros::Publisher cone_pub_;
  ros::Publisher lidar_cone_pub_;
  ros::Publisher carstate_pub_;

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

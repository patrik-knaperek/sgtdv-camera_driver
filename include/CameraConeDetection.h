/*****************************************************/
//Organization: Stuba Green Team
//Authors: Matúš Tomšík, Juraj Krasňanský
/*****************************************************/

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <sgtdv_msgs/ConeArr.h>
#include <sgtdv_msgs/Cone.h>
#include <sgtdv_msgs/Point2D.h>
#include <sgtdv_msgs/Point2DArr.h>
#include <sgtdv_msgs/CarState.h>
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

constexpr int FPS = 2;
constexpr int TIME_PER_FRAME = 1000 / FPS;

class CameraConeDetection {
public:
    CameraConeDetection();

    ~CameraConeDetection();

    void SetSignalPublisher(ros::Publisher signalPublisher);
    void SetConePublisher(ros::Publisher mainPublisher);

#ifdef DEBUG_STATE
    void SetVisDebugPublisher(ros::Publisher visDebugPublisher) { m_visDebugPublisher = visDebugPublisher; }
#endif

#ifdef CAMERA_DETECTION_FAKE_LIDAR

    void SetLidarConePublisher(ros::Publisher lidarConePublisher);

#endif//CAMERA_DETECTION_FAKE_LIDAR
#ifdef CAMERA_DETECTION_CARSTATE

    void SetCarStatePublisher(ros::Publisher carStatePublisher);

#endif//CAMERA_DETECTION_CARSTATE

    void Do();

    void predict(Detector &detector, sl::MODEL &cam_model);

private:
    std::string names_file = "kuzel.names";
    std::string cfg_file = "yolov3-tiny.cfg";
    std::string weights_file = "yolov3-tiny.weights";
    float const thresh = 0.2;
    //std::string filename = "druha_jazda.svo";
    std::string filename = "zed_camera";
    sl::Camera zed; // ZED-camera

    std::string out_videofile = "result.avi";
    cv::VideoWriter output_video;

    ros::Publisher m_signalPublisher;
    ros::Publisher m_conePublisher;
#ifdef CAMERA_DETECTION_FAKE_LIDAR
    ros::Publisher m_lidarConePublisher;
#endif//CAMERA_DETECTION_FAKE_LIDAR
#ifdef CAMERA_DETECTION_CARSTATE
    ros::Publisher m_carStatePublisher;
#endif//CAMERA_DETECTION_CARSTATE

#ifdef DEBUG_STATE
    ros::Publisher m_visDebugPublisher;
    size_t m_numOfDetectedCones = 0;
#endif

    float getMedian(std::vector<float> &v);

    std::vector <bbox_t> get_3d_coordinates(std::vector <bbox_t> bbox_vect, cv::Mat xyzrgba);

    cv::Mat slMat2cvMat(sl::Mat &input);

    cv::Mat zed_capture_rgb(sl::Camera &zed);

    cv::Mat zed_capture_3d(sl::Camera &zed);

    void
    show_console_result(std::vector <bbox_t> const result_vec, std::vector <std::string> const obj_names, int frame_id);

    cv::Mat draw_boxes(cv::Mat mat_img, std::vector <bbox_t> result_vec, std::vector <std::string> obj_names,
                       int current_det_fps, int current_cap_fps);
//    std::vector<std::string> objects_names_from_file(std::string const filename);
};

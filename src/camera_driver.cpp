/*****************************************************/
//Organization: Stuba Green Team
//Authors: Matúš Tomšík, Juraj Krasňanský, Patrik Knaperek
/*****************************************************/

/* ROS */
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Imu.h>

/* Header */
#include "camera_driver.h"

CameraDriver::CameraDriver(ros::NodeHandle& handle, const CameraConeDetection::Params& nn_params)
  : camera_cone_detection_(nn_params)

  /* ROS Interface initialization */
  , cone_pub_(handle.advertise<sgtdv_msgs::ConeStampedArr>("camera/cones", 1))
  , carstate_pub_(handle.advertise<geometry_msgs::PoseWithCovarianceStamped>("camera/pose", 1))
  , imu_pub_(handle.advertise<sensor_msgs::Imu>("camera/imu", 1))
  , reset_odom_server_(handle.advertiseService("camera/reset_odometry", &CameraDriver::resetOdomCallback, this))
#ifdef SGT_DEBUG_STATE
  , vis_debug_pub_(handle.advertise<sgtdv_msgs::DebugState>("camera/debug_state", 1))
#endif
{
  loadParams(handle);
  initialize();
}

CameraDriver::~CameraDriver()
{
  if(params_.record_video_svo)
    zed_.disableRecording();

  if (zed_.isOpened())
  {
    zed_.close();
    ROS_INFO("zed camera closed");
  }
  
  if(params_.record_video) output_video_.release();
}

void CameraDriver::loadParams(const ros::NodeHandle& nh)
{
  const auto path_to_package = ros::package::getPath("camera_driver");
  std::string filename_temp;
  
  Utils::loadParam(nh, "/obj_names_filename", &filename_temp);
  params_.names_file = path_to_package + filename_temp;
  
  Utils::loadParam(nh, "/output_video_filename", &filename_temp);
  params_.out_video_file = path_to_package + filename_temp;
  
  Utils::loadParam(nh, "/output_svo_filename", &filename_temp);
  params_.out_svo_file = path_to_package + filename_temp;
  
  Utils::loadParam(nh, "/input_stream", &filename_temp);
  params_.in_svo_file = path_to_package + filename_temp;

  Utils::loadParam(nh, "/publish_carstate", false, &params_.publish_carstate);
  Utils::loadParam(nh, "/camera_show", false, &params_.camera_show);
  Utils::loadParam(nh, "/console_show", false, &params_.console_show);
  Utils::loadParam(nh, "/record_video", false, &params_.record_video);
  Utils::loadParam(nh, "/record_video_svo", false, &params_.record_video_svo);
};

void CameraDriver::initialize(void)
{  
  camera_cone_detection_.getObjectsNamesFromFile(params_.names_file);

  // get ZED SDK version
  int major_dll, minor_dll, patch_dll;
  getZEDSDKBuildVersion(major_dll, minor_dll, patch_dll);
  if (major_dll < 3)
  {
    ROS_ERROR("SUPPORT ONLY SDK 3.*");
    ros::shutdown();
    return;
  }

  // init zed camera
  sl::InitParameters init_params;
  init_params.depth_minimum_distance = 1.7;
  init_params.depth_mode = sl::DEPTH_MODE::ULTRA;
  init_params.camera_resolution = sl::RESOLUTION::HD720;// sl::RESOLUTION::HD1080, sl::RESOLUTION::HD720
  init_params.coordinate_units = sl::UNIT::METER;
  init_params.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP_X_FWD;  /*< Right-Handed with Z pointing up and X forward. Used in ROS (REP 103). */
  init_params.sdk_cuda_ctx = (CUcontext) camera_cone_detection_.getDetectorCudaContext();//ak to bude nadavat na CUDNN tak treba zakomentova/odkomentovat
  init_params.sdk_gpu_id = camera_cone_detection_.getGPUid();
  //init_params.camera_buffer_count_linux = 2;

  /* detect SVO input type demand */
  if (params_.in_svo_file.size() == 0)
  {
    ROS_ERROR("Parameter \"input_stream\" must not be empty!");
    ros::shutdown();
    return;
  }
  std::string const file_ext = params_.in_svo_file.substr(params_.in_svo_file.find_last_of(".") + 1);
  if (file_ext == "svo") init_params.input.setFromSVOFile(params_.in_svo_file.c_str());
  
  /* open input stream */
  sl::ERROR_CODE zed_open = zed_.open(init_params);
  if (zed_open != sl::ERROR_CODE::SUCCESS)
  {
    ROS_ERROR_STREAM("ZED OPEN ERROR: " << zed_open);
    ros::shutdown();
    return;
  }

  if (!zed_.isOpened())
  {
    ROS_ERROR(" Error: ZED Camera should be connected to USB 3.0. And ZED_SDK should be installed. \n");
    ros::shutdown();
    return;
  }

  // Check camera model
  cam_model_ = zed_.getCameraInformation().camera_model;
  ROS_INFO_STREAM("camera model: " << cam_model_);

  /* set additional camera settings */
  zed_.setCameraSettings(sl::VIDEO_SETTINGS::WHITEBALANCE_AUTO, 1);

  /* enable SVO output recording */
  if(params_.record_video_svo)
  {
    sl::ERROR_CODE err 
      = zed_.enableRecording(sl::RecordingParameters(params_.out_svo_file.c_str(), sl::SVO_COMPRESSION_MODE::H264));
    if (err != sl::ERROR_CODE::SUCCESS)
    {
      ROS_ERROR_STREAM("CAMERA_DETECTION_RECORD_VIDEO_SVO: " <<  sl::toString(err));
    }
  }

  if(params_.publish_carstate)
  {
    // Set parameters for Positional Tracking
    sl::PositionalTrackingParameters tracking_parameters;
    tracking_parameters.enable_area_memory = false; //disable to use pose_covariance
    zed_.enablePositionalTracking(tracking_parameters);
  }

  if(params_.record_video)
  {
  #ifdef CV_VERSION_EPOCH // OpenCV 2.x
    output_video_.open(out_videofile_, CV_FOURCC('D', 'I', 'V', 'X'), FPS, cv::Size(1280, 720), true);
  #else
    output_video_.open(params_.out_video_file, cv::VideoWriter::fourcc('D', 'I', 'V', 'X'), FPS, cv::Size(1280, 720), 
                      true);
  #endif
  }
}

void CameraDriver::update()
{
  #ifdef SGT_DEBUG_STATE
    sgtdv_msgs::DebugState state;
    state.stamp = ros::Time::now();
    state.working_state = 1;
    vis_debug_pub_.publish(state);
  #endif

  if (zed_.grab() == sl::ERROR_CODE::SUCCESS)
  {
    const auto capture_time = ros::Time::now();
    auto cur_frame = zedCaptureRGB();
    const auto zed_cloud = zedCapture3D();
    if (cur_frame.empty())
    {
      ROS_WARN_STREAM("exit_flag: detection_data.cap_frame.size = " << cur_frame.size());
      cur_frame = cv::Mat(cur_frame.size(), CV_8UC3);
    }

    /*************** CAMERA CONE DETECTION ***************/
    auto result_vec = camera_cone_detection_.detect(cur_frame, zed_cloud);
      
  #ifdef SGT_DEBUG_STATE
    num_of_detected_cones_ = result_vec.size();
  #endif

    { /* Fill up camera/cones topic message */
      sgtdv_msgs::ConeStampedArr coneArr;
      sgtdv_msgs::ConeStamped cone;
      cone.coords.header.frame_id = "camera_left";
      cone.coords.header.stamp = capture_time;
  
      int i_n = 0;
      for (const auto &i : result_vec)
      {
        cone.coords.header.seq = i_n++;
        cone.coords.x = i.x_3d;
        cone.coords.y = i.y_3d;
        
        switch(i.obj_id)
        {
          case CameraConeDetection::CONE_CLASSES::YELLOW       : cone.color = 'y'; break;
          case CameraConeDetection::CONE_CLASSES::BLUE         : cone.color = 'b'; break;
          case CameraConeDetection::CONE_CLASSES::ORANGE_SMALL : cone.color = 's'; break;
          case CameraConeDetection::CONE_CLASSES::ORANGE_BIG   : cone.color = 'g'; break;
          default                                              : break;
        }
        coneArr.cones.push_back(cone);
      }
      cone_pub_.publish(coneArr);
    }

  #ifdef SGT_DEBUG_STATE
    state.working_state = 0;
    state.stamp = ros::Time::now();
    state.num_of_cones = static_cast<uint32_t>(num_of_detected_cones_);
    vis_debug_pub_.publish(state);
  #endif

    if(params_.camera_show || params_.record_video)
    {
      camera_cone_detection_.drawBoxes(&cur_frame, result_vec);
      
      if(params_.camera_show)
      {
        cv::imshow("window name", cur_frame);
      }

      if(params_.record_video)
      {
        output_video_ << cur_frame;
      }
    }

    if(params_.console_show) camera_cone_detection_.showConsoleResult(result_vec);

    
    /*************** CAMERA POSE ESTIMATION ***************/

    if(params_.publish_carstate)
    { /* Fill up camera_pose topic message */
      const auto tracking_state = zed_.getPosition(camera_pose_, sl::REFERENCE_FRAME::WORLD); //get actual position

      geometry_msgs::PoseWithCovarianceStamped car_state;
      car_state.header.stamp = capture_time;
      car_state.header.frame_id = "odom";

      car_state.pose.pose.position.x = camera_pose_.getTranslation().x;
      car_state.pose.pose.position.y = camera_pose_.getTranslation().y;
      car_state.pose.pose.position.z = camera_pose_.getTranslation().z;

      car_state.pose.pose.orientation.x = camera_pose_.getOrientation().x;
      car_state.pose.pose.orientation.y = camera_pose_.getOrientation().y;
      car_state.pose.pose.orientation.z = camera_pose_.getOrientation().z;
      car_state.pose.pose.orientation.w = camera_pose_.getOrientation().w;

      for (size_t i = 0; i < (sizeof(camera_pose_.pose_covariance)/sizeof(*camera_pose_.pose_covariance)); i++)
      {
        car_state.pose.covariance[i] = camera_pose_.pose_covariance[i];
      }

      carstate_pub_.publish(car_state);
    }    
    /*************** ZED2 IMU INTERFACE ***************/

    if (cam_model_ >= sl::MODEL::ZED2) 
    {
      sensor_msgs::Imu imu_data;
      imu_data.header.stamp = capture_time;
      imu_data.header.frame_id = "camera_center";
      
      if (zed_.getSensorsData(sensors_data_, sl::TIME_REFERENCE::CURRENT) == sl::ERROR_CODE::SUCCESS)
      {
        // Filtered orientation quaternion
        ROS_DEBUG_STREAM("IMU Orientation x: " << sensors_data_.imu.pose.getOrientation().ox << 
                      "y: " << sensors_data_.imu.pose.getOrientation().oy <<
                      "z: " << sensors_data_.imu.pose.getOrientation().oz << 
                      "w: " << sensors_data_.imu.pose.getOrientation().ow
        );

        imu_data.orientation.x = sensors_data_.imu.pose.getOrientation().ox;
        imu_data.orientation.y = sensors_data_.imu.pose.getOrientation().oy;
        imu_data.orientation.z = sensors_data_.imu.pose.getOrientation().oz;
        imu_data.orientation.w = sensors_data_.imu.pose.getOrientation().ow;
        imu_data.orientation_covariance = 
        {
          sensors_data_.imu.pose_covariance.r00,
          sensors_data_.imu.pose_covariance.r01,
          sensors_data_.imu.pose_covariance.r02,
          sensors_data_.imu.pose_covariance.r10,
          sensors_data_.imu.pose_covariance.r11,
          sensors_data_.imu.pose_covariance.r12,
          sensors_data_.imu.pose_covariance.r20,
          sensors_data_.imu.pose_covariance.r21,
          sensors_data_.imu.pose_covariance.r22
        };

        // Filtered acceleration
        ROS_DEBUG_STREAM("IMU Acceleration [m/sec^2] x: " << sensors_data_.imu.linear_acceleration.x << 
                        "y: " << sensors_data_.imu.linear_acceleration.y <<
                        "z: " << sensors_data_.imu.linear_acceleration.z
        );

        imu_data.linear_acceleration.x = sensors_data_.imu.linear_acceleration.x;
        imu_data.linear_acceleration.y = sensors_data_.imu.linear_acceleration.y;
        imu_data.linear_acceleration.z = sensors_data_.imu.linear_acceleration.z;
        imu_data.linear_acceleration_covariance = 
        {
          sensors_data_.imu.linear_acceleration_covariance.r00,
          sensors_data_.imu.linear_acceleration_covariance.r01,
          sensors_data_.imu.linear_acceleration_covariance.r02,
          sensors_data_.imu.linear_acceleration_covariance.r10,
          sensors_data_.imu.linear_acceleration_covariance.r11,
          sensors_data_.imu.linear_acceleration_covariance.r12,
          sensors_data_.imu.linear_acceleration_covariance.r20,
          sensors_data_.imu.linear_acceleration_covariance.r21,
          sensors_data_.imu.linear_acceleration_covariance.r22
        };

        // Filtered angular velocities
        ROS_DEBUG_STREAM("IMU angular velocities [deg/sec] x: " << sensors_data_.imu.angular_velocity.x << 
                        "y: " << sensors_data_.imu.angular_velocity.y <<
                        "z: " << sensors_data_.imu.angular_velocity.z
        );

        imu_data.angular_velocity.x = sensors_data_.imu.angular_velocity.x * M_PI / 180.0;
        imu_data.angular_velocity.y = sensors_data_.imu.angular_velocity.y * M_PI / 180.0;
        imu_data.angular_velocity.z = sensors_data_.imu.angular_velocity.z * M_PI / 180.0;
        imu_data.angular_velocity_covariance = 
        {
          sensors_data_.imu.angular_velocity_covariance.r00,
          sensors_data_.imu.angular_velocity_covariance.r01,
          sensors_data_.imu.angular_velocity_covariance.r02,
          sensors_data_.imu.angular_velocity_covariance.r10,
          sensors_data_.imu.angular_velocity_covariance.r11,
          sensors_data_.imu.angular_velocity_covariance.r12,
          sensors_data_.imu.angular_velocity_covariance.r20,
          sensors_data_.imu.angular_velocity_covariance.r21,
          sensors_data_.imu.angular_velocity_covariance.r22
        };
      }

      imu_pub_.publish(imu_data);
    }
  }
}

bool CameraDriver::resetOdomCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  zed_.resetPositionalTracking(sl::Transform(sl::Matrix4f::identity()));

  return true;
}

cv::Mat CameraDriver::slMat2cvMat(const sl::Mat &input) const
{
  // Mapping between MAT_TYPE and CV_TYPE
  int cv_type = -1;
  switch (input.getDataType())
  {
    case sl::MAT_TYPE::F32_C1 : cv_type = CV_32FC1; break;
    case sl::MAT_TYPE::F32_C2 : cv_type = CV_32FC2; break;
    case sl::MAT_TYPE::F32_C3 : cv_type = CV_32FC3; break;
    case sl::MAT_TYPE::F32_C4 : cv_type = CV_32FC4; break;
    case sl::MAT_TYPE::U8_C1  : cv_type = CV_8UC1; break;
    case sl::MAT_TYPE::U8_C2  : cv_type = CV_8UC2; break;
    case sl::MAT_TYPE::U8_C3  : cv_type = CV_8UC3; break;
    case sl::MAT_TYPE::U8_C4  : cv_type = CV_8UC4; break;
    default                   : break;
  }
  return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(sl::MEM::CPU));
}

cv::Mat CameraDriver::zedCaptureRGB(void)
{
  sl::Mat left;
  zed_.retrieveImage(left);
  cv::Mat left_rgb;
  cv::cvtColor(slMat2cvMat(left), left_rgb, cv::COLOR_RGBA2RGB);
  return left_rgb;
}

cv::Mat CameraDriver::zedCapture3D(void)
{
  sl::Mat cur_cloud;
  zed_.retrieveMeasure(cur_cloud, sl::MEASURE::XYZ);
  return slMat2cvMat(cur_cloud).clone();
}

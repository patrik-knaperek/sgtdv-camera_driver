/*****************************************************/
//Organization: Stuba Green Team
//Authors: Matúš Tomšík, Juraj Krasňanský, Patrik Knaperek
/*****************************************************/

#include "camera_driver.h"

CameraDriver::CameraDriver(ros::NodeHandle& handle, const Params& params)
  : detector_(params.cfg_file, params.weights_file)
  , params_(params)
  , cone_pub_(handle.advertise<sgtdv_msgs::ConeStampedArr>("camera_cones", 1))
  , carstate_pub_(handle.advertise<geometry_msgs::PoseWithCovarianceStamped>("camera_pose", 1))
  , reset_odom_sub_(handle.subscribe("reset_odometry", 1, &CameraDriver::resetOdomCallback, this))
#ifdef SGT_DEBUG_STATE
  , vis_debug_pub_(handle.advertise<sgtdv_msgs::DebugState>("camera_driver_debug_state", 1))
#endif
{
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

void CameraDriver::resetOdomCallback(const std_msgs::Empty::ConstPtr& msg)
{
  zed_.resetPositionalTracking(sl::Transform(sl::Matrix4f::identity()));
}

void CameraDriver::initialize(void)
{  
  getObjectsNamesFromFile();
  if (obj_names_.size() == 0)
  {
    ROS_ERROR("Unable to obtain object names.");
    ros::shutdown();
    return;
  }

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
  init_params.sdk_cuda_ctx = (CUcontext) detector_.get_cuda_context();//ak to bude nadavat na CUDNN tak treba zakomentova/odkomentovat
  init_params.sdk_gpu_id = detector_.cur_gpu_id;
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
  sl::MODEL cam_model = zed_.getCameraInformation().camera_model;

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

    std::vector <bbox_t> result_vec = detector_.detect(cur_frame, DETECT_TH);
    get3dCoordinates(&result_vec, zed_cloud);
      
  #ifdef SGT_DEBUG_STATE
    num_of_detected_cones_ = result_vec.size();
  #endif

    { /* Fill up camera_cones topic message */
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
          case CONE_CLASSES::YELLOW       : cone.color = 'y'; break;
          case CONE_CLASSES::BLUE         : cone.color = 'b'; break;
          case CONE_CLASSES::ORANGE_SMALL : cone.color = 's'; break;
          case CONE_CLASSES::ORANGE_BIG   : cone.color = 'g'; break;
          default                         : break;
        }
        coneArr.cones.push_back(cone);
      }
      cone_pub_.publish(coneArr);
    }

    if(params_.publish_carstate)
    { /* Fill up camera_pose topic message */
      const auto tracking_state = zed_.getPosition(camera_pose_, sl::REFERENCE_FRAME::WORLD); //get actual position

      geometry_msgs::PoseWithCovarianceStamped carState;
      carState.header.stamp = capture_time;
      carState.header.frame_id = "odom";

      carState.pose.pose.position.x = camera_pose_.getTranslation().x;
      carState.pose.pose.position.y = camera_pose_.getTranslation().y;
      carState.pose.pose.position.z = camera_pose_.getTranslation().z;

      carState.pose.pose.orientation.x = camera_pose_.getOrientation().x;
      carState.pose.pose.orientation.y = camera_pose_.getOrientation().y;
      carState.pose.pose.orientation.z = camera_pose_.getOrientation().z;
      carState.pose.pose.orientation.w = camera_pose_.getOrientation().w;

      for (size_t i = 0; i < (sizeof(camera_pose_.pose_covariance)/sizeof(*camera_pose_.pose_covariance)); i++)
      {
        carState.pose.covariance[i] = camera_pose_.pose_covariance[i];
      }

      carstate_pub_.publish(carState);
    }    

    // if (cam_model_ == sl::MODEL::ZED2) 
    // {
    //   if (zed_.getSensorsData(sensors_data_, sl::TIME_REFERENCE::CURRENT) == sl::ERROR_CODE::SUCCESS)
    //   {
    //     // Filtered orientation quaternion
    //     std::cout << "IMU Orientation x: " << sensors_data_.imu.pose.getOrientation().ox << "y: "
    //               << sensors_data_.imu.pose.getOrientation().oy <<
    //               "z: " << sensors_data_.imu.pose.getOrientation().oz << "w: " << sensors_data_.imu.pose.getOrientation().ow
    //               << std::endl;

    //     // Filtered acceleration
    //     std::cout << "IMU Acceleration [m/sec^2] x: " << sensors_data_.imu.linear_acceleration.x << "y: "
    //               << sensors_data_.imu.linear_acceleration.y <<
    //               "z: " << sensors_data_.imu.linear_acceleration.z << std::endl;

    //     // Filtered angular velocities
    //     std::cout << "IMU angular velocities [deg/sec] x: " << sensors_data_.imu.angular_velocity.x << "y: "
    //               << sensors_data_.imu.angular_velocity.y <<
    //               "z: " << sensors_data_.imu.angular_velocity.z << std::endl;
    //   }
    // }

    if(params_.camera_show || params_.record_video)
    {
      drawBoxes(&cur_frame, result_vec);

      if(params_.record_video)
      {
        output_video_ << cur_frame;
      }
    }

    if(params_.console_show) showConsoleResult(result_vec);
  }
    
  #ifdef SGT_DEBUG_STATE
    state.working_state = 0;
    state.stamp = ros::Time::now();
    state.num_of_cones = static_cast<uint32_t>(num_of_detected_cones_);
    vis_debug_pub_.publish(state);
  #endif
}

void CameraDriver::drawBoxes(cv::Mat* mat_img, const std::vector <bbox_t>& result_vec,
                                    const int current_det_fps, const int current_cap_fps) const
{
  for (const auto &i : result_vec)
  {
    /* set rectangle color by cone class ID */
    cv::Scalar color = {255, 255, 255}; // BGR
    switch(i.obj_id)
    {
      case CONE_CLASSES::YELLOW       : color = {0, 255, 255}; break;
      case CONE_CLASSES::BLUE         : color = {255, 0, 0}; break;
      case CONE_CLASSES::ORANGE_SMALL : color = {0, 120, 255}; break;
      case CONE_CLASSES::ORANGE_BIG   : color = {0, 80, 255}; break;
      default                         : break;
    }
      
    /* create bounding box visualization */
    cv::rectangle(*mat_img, cv::Rect(i.x, i.y, i.w, i.h), color, 2);
    if (obj_names_.size() > i.obj_id)
    { /* write object name */
      std::string obj_name = obj_names_[i.obj_id];
      if (i.track_id > 0) obj_name += " - " + std::to_string(i.track_id);
      
      cv::Size const text_size = cv::getTextSize(obj_name, cv::FONT_HERSHEY_COMPLEX_SMALL, 1.2, 2, 0);
      int max_width = std::max(text_size.width, static_cast<int>(i.w + 2));
      
      /* write cone coords */
      std::string coords_3d;
      if (!std::isnan(i.z_3d))
      {
        std::stringstream ss;
        ss << std::fixed << std::setprecision(3) << "x:" << i.x_3d << " y:" << i.y_3d << " z:" << i.z_3d;
        coords_3d = ss.str();
        const cv::Size text_size_3d = getTextSize(coords_3d, cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, 1, 0);
        const int max_width_3d = std::max(text_size_3d.width, static_cast<int>(i.w + 2));
        max_width = std::max(max_width, max_width_3d);
      }

      cv::rectangle(*mat_img, cv::Point2f(std::max(static_cast<int>(i.x) - 1, 0),
                                          std::max(static_cast<int>(i.y) - 35, 0)),
                    cv::Point2f(std::min(static_cast<int>(i.x) + max_width, mat_img->cols - 1),
                                std::min(static_cast<int>(i.y), mat_img->rows - 1)),
                    color, CV_FILLED, 8, 0);
      cv::putText(*mat_img, obj_name, cv::Point2f(i.x, i.y - 16), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.2,
              cv::Scalar(0, 0, 0), 2);
      if (!coords_3d.empty())
          cv::putText(*mat_img, coords_3d, cv::Point2f(i.x, i.y - 1), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8,
                  cv::Scalar(0, 0, 0), 1);
    }
  }
  
  if (current_det_fps >= 0 && current_cap_fps >= 0)
  {
    std::string fps_str = "FPS detection: " + std::to_string(current_det_fps) + "   FPS capture: "
                        + std::to_string(current_cap_fps);
    putText(*mat_img, fps_str, cv::Point2f(10, 20), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.2, cv::Scalar(50, 255, 0), 2);
  }

  if(params_.camera_show)
    cv::imshow("window name", *mat_img);
  cv::waitKey(3);
}

void CameraDriver::getObjectsNamesFromFile(void)
{
	std::ifstream file(params_.names_file);
	if (!file.is_open())
  {
    ROS_ERROR_STREAM("Could not open file " << params_.names_file);
    ros::shutdown();
    return;
  }

	for(std::string line; file >> line;)
    obj_names_.push_back(line);
	
  ROS_INFO("object names loaded");	
}

void CameraDriver::showConsoleResult(const std::vector<bbox_t>& result_vec) const
{
  for (const auto &i : result_vec)
  {
    if (obj_names_.size() > i.obj_id) 
      std::cout << obj_names_[i.obj_id] << " - ";
    
    const std::string obj_name = obj_names_[i.obj_id];
    std::cout << "obj_name = " << obj_name << ",  x = " << i.x_3d << ", y = " << i.y_3d
      << std::setprecision(3) << ", prob = " << i.prob << std::endl;
  }
}

float CameraDriver::getMedian(std::vector<float> &v) const
{
  const size_t n = v.size() / 2;
  std::nth_element(v.begin(), v.begin() + n, v.end());
  return v[n];
}

void CameraDriver::get3dCoordinates(std::vector <bbox_t>* bbox_vect, const cv::Mat& xyzrgba) const
{
  int i, j;
  static const unsigned int R_max_global = 10;

  std::vector <bbox_t> bbox3d_vect;

  for (auto &cur_box : *bbox_vect)
  {

    const unsigned int obj_size = std::min(cur_box.w, cur_box.h);
    const unsigned int R_max = std::min(R_max_global, obj_size / 2);
    const int center_i = cur_box.x + cur_box.w * 0.5f;
    const int center_j = cur_box.y + cur_box.h * 0.5f;

    std::vector<float> x_vect, y_vect, z_vect;
    for (int R = 0; R < R_max; R++)
    {
      for (int y = -R; y <= R; y++)
      {
        for (int x = -R; x <= R; x++)
        {
          i = center_i + x;
          j = center_j + y;
          sl::float4 out(NAN, NAN, NAN, NAN);
          if (i >= 0 && i < xyzrgba.cols && j >= 0 && j < xyzrgba.rows)
          {
            cv::Vec4f elem = xyzrgba.at<cv::Vec4f>(j, i);  // x,y,z,w
            out.x = elem[0];
            out.y = elem[1];
            out.z = elem[2];
            out.w = elem[3];
          }
          if (std::isfinite(out.z)) // valid measure
          {
            x_vect.push_back(out.x);
            y_vect.push_back(out.y);
            z_vect.push_back(out.z);
          }
        }
      }
    }

    if (x_vect.size() * y_vect.size() * z_vect.size() > 0)
    {
      cur_box.x_3d = getMedian(x_vect);
      cur_box.y_3d = getMedian(y_vect);
      cur_box.z_3d = getMedian(z_vect);
    } 
    else 
    {
      cur_box.x_3d = NAN;
      cur_box.y_3d = NAN;
      cur_box.z_3d = NAN;
    }

    bbox3d_vect.emplace_back(cur_box);
  }

  bbox_vect = &bbox3d_vect;
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

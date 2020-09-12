/*****************************************************/
//Organization: Stuba Green Team
//Authors: Matúš Tomšík, Juraj Krasňanský
/*****************************************************/


#include "../include/CameraConeDetection.h"

CameraConeDetection::CameraConeDetection()
{

}

CameraConeDetection::~CameraConeDetection()
{
#ifdef CAMERA_DETECTION_RECORD_VIDEO_SVO
    zed.disableRecording();
#endif// CAMERA_DETECTION_RECORD_VIDEO_SVO

    if (zed.isOpened()) {
        zed.close();
        std::cout << "zed camera closed" << std::endl;
    }
#ifdef CAMERA_DETECTION_RECORD_VIDEO
    output_video.release();
#endif //CAMERA_DETECTION_RECORD_VIDEO
}

void CameraConeDetection::SetSignalPublisher(ros::Publisher signalPublisher)
{
    m_signalPublisher = signalPublisher;
}

void CameraConeDetection::SetConePublisher(ros::Publisher conePublisher)
{
    m_conePublisher = conePublisher;
}
#ifdef CAMERA_DETECTION_FAKE_LIDAR
    void CameraConeDetection::SetLidarConePublisher(ros::Publisher lidarConePublisher)
    {
        m_lidarConePublisher = lidarConePublisher;
    }
#endif//CAMERA_DETECTION_FAKE_LIDAR

#ifdef CAMERA_DETECTION_CARSTATE
void CameraConeDetection::SetCarStatePublisher(ros::Publisher carStatePublisher)
{
    m_carStatePublisher = carStatePublisher;
}
#endif//CAMERA_DETECTION_CARSTATE

#ifdef CAMERA_DETECTION_CAMERA_SHOW
cv::Mat CameraConeDetection::draw_boxes(cv::Mat mat_img, std::vector<bbox_t> result_vec, std::vector<std::string> obj_names, int current_det_fps = -1, int current_cap_fps = -1) {
    for (auto &i : result_vec) {
        cv::Scalar color = { 0,0,255 };
        if (i.obj_id == 0){
            color = { 0,255,255 };
        } else if (i.obj_id == 1){
            color = { 255,0,0 };
        }
        cv::rectangle(mat_img, cv::Rect(i.x, i.y, i.w, i.h), color, 2);
        if (obj_names.size() > i.obj_id) {
            std::string obj_name = obj_names[i.obj_id];
            if (i.track_id > 0) obj_name += " - " + std::to_string(i.track_id);
            cv::Size const text_size = getTextSize(obj_name, cv::FONT_HERSHEY_COMPLEX_SMALL, 1.2, 2, 0);
            int max_width = (text_size.width > i.w + 2) ? text_size.width : (i.w + 2);
            max_width = std::max(max_width, (int)i.w + 2);
            //max_width = std::max(max_width, 283);
            std::string coords_3d;
            if (!std::isnan(i.z_3d)) {
                std::stringstream ss;
                ss << std::fixed << std::setprecision(2) << "x:" << i.x_3d << "mm y:" << i.y_3d << "mm z:" << i.z_3d << "mm ";
                coords_3d = ss.str();
                cv::Size const text_size_3d = getTextSize(ss.str(), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, 1, 0);
                int const max_width_3d = (text_size_3d.width > i.w + 2) ? text_size_3d.width : (i.w + 2);
                if (max_width_3d > max_width) max_width = max_width_3d;
            }

            cv::rectangle(mat_img, cv::Point2f(std::max((int)i.x - 1, 0), std::max((int)i.y - 35, 0)),
                cv::Point2f(std::min((int)i.x + max_width, mat_img.cols - 1), std::min((int)i.y, mat_img.rows - 1)),
                color, CV_FILLED, 8, 0);
            putText(mat_img, obj_name, cv::Point2f(i.x, i.y - 16), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.2, cv::Scalar(0, 0, 0), 2);
            if(!coords_3d.empty()) putText(mat_img, coords_3d, cv::Point2f(i.x, i.y-1), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(0, 0, 0), 1);
        }
    }
    if (current_det_fps >= 0 && current_cap_fps >= 0) {
        std::string fps_str = "FPS detection: " + std::to_string(current_det_fps) + "   FPS capture: " + std::to_string(current_cap_fps);
        putText(mat_img, fps_str, cv::Point2f(10, 20), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.2, cv::Scalar(50, 255, 0), 2);
    }

    cv::imshow("window name", mat_img);
    cv::waitKey(3);
    return mat_img;
}
#endif //CAMERA_DETECTION_CAMERA_SHOW

std::vector<std::string> CameraConeDetection::objects_names_from_file(std::string const filename) {
	std::ifstream file(filename);
	std::vector<std::string> file_lines;
	if (!file.is_open()) return file_lines;
	for(std::string line; file >> line;) file_lines.push_back(line);
	std::cout << "object names loaded \n";
	return file_lines;
}

#ifdef CAMERA_DETECTION_CONSOLE_SHOW
    void CameraConeDetection::show_console_result(std::vector<bbox_t> const result_vec, std::vector<std::string> const obj_names, int frame_id = -1) {
        if (frame_id >= 0) std::cout << " Frame: " << frame_id << std::endl;
        for (auto &i : result_vec) {
            if (obj_names.size() > i.obj_id) std::cout << obj_names[i.obj_id] << " - ";
            std::string obj_name = obj_names[i.obj_id];
            std::cout << "obj_name = " << obj_name << ",  x = " << i.x_3d << ", y = " << i.y_3d
                << std::setprecision(3) << ", prob = " << i.prob << std::endl;
        }
    }
#endif //CAMERA_DETECTION_CONSOLE_SHOW
float CameraConeDetection::getMedian(std::vector<float> &v) {
    size_t n = v.size() / 2;
    std::nth_element(v.begin(), v.begin() + n, v.end());
    return v[n];
}

std::vector<bbox_t> CameraConeDetection::get_3d_coordinates(std::vector<bbox_t> bbox_vect, cv::Mat xyzrgba)
{
    bool valid_measure;
    int i, j;
    const unsigned int R_max_global = 10;

    std::vector<bbox_t> bbox3d_vect;

    for (auto &cur_box : bbox_vect) {

        const unsigned int obj_size = std::min(cur_box.w, cur_box.h);
        const unsigned int R_max = std::min(R_max_global, obj_size / 2);
        int center_i = cur_box.x + cur_box.w * 0.5f, center_j = cur_box.y + cur_box.h * 0.5f;

        std::vector<float> x_vect, y_vect, z_vect;
        for (int R = 0; R < R_max; R++) {
            for (int y = -R; y <= R; y++) {
                for (int x = -R; x <= R; x++) {
                    i = center_i + x;
                    j = center_j + y;
                    sl::float4 out(NAN, NAN, NAN, NAN);
                    if (i >= 0 && i < xyzrgba.cols && j >= 0 && j < xyzrgba.rows) {
                        cv::Vec4f &elem = xyzrgba.at<cv::Vec4f>(j, i);  // x,y,z,w
                        out.x = elem[0];
                        out.y = elem[1];
                        out.z = elem[2];
                        out.w = elem[3];
                    }
                    valid_measure = std::isfinite(out.z);
                    if (valid_measure)
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
        else {
            cur_box.x_3d = NAN;
            cur_box.y_3d = NAN;
            cur_box.z_3d = NAN;
        }

        bbox3d_vect.emplace_back(cur_box);
    }

    return bbox3d_vect;
}


cv::Mat CameraConeDetection::slMat2cvMat(sl::Mat &input) {
    // Mapping between MAT_TYPE and CV_TYPE
    int cv_type = -1;
    switch (input.getDataType()) {
    case sl::MAT_TYPE::F32_C1:
        cv_type = CV_32FC1;
        break;
    case sl::MAT_TYPE::F32_C2:
        cv_type = CV_32FC2;
        break;
    case sl::MAT_TYPE::F32_C3:
        cv_type = CV_32FC3;
        break;
    case sl::MAT_TYPE::F32_C4:
        cv_type = CV_32FC4;
        break;
    case sl::MAT_TYPE::U8_C1:
        cv_type = CV_8UC1;
        break;
    case sl::MAT_TYPE::U8_C2:
        cv_type = CV_8UC2;
        break;
    case sl::MAT_TYPE::U8_C3:
        cv_type = CV_8UC3;
        break;
    case sl::MAT_TYPE::U8_C4:
        cv_type = CV_8UC4;
        break;
    default:
        break;
    }
    return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(sl::MEM::CPU));
}

cv::Mat CameraConeDetection::zed_capture_rgb(sl::Camera &zed) {
    sl::Mat left;
    zed.retrieveImage(left);
    cv::Mat left_rgb;
    cv::cvtColor(slMat2cvMat(left), left_rgb, cv::COLOR_RGBA2RGB);
    return left_rgb;
}

cv::Mat CameraConeDetection::zed_capture_3d(sl::Camera &zed) {
    sl::Mat cur_cloud;
    zed.retrieveMeasure(cur_cloud, sl::MEASURE::XYZ);
    return slMat2cvMat(cur_cloud).clone();
}


void CameraConeDetection::Do()
{
    Detector detector(cfg_file, weights_file); //Darknet
//    auto obj_names = objects_names_from_file(names_file);

    std::string const file_ext = filename.substr(filename.find_last_of(".") + 1);
    std::string const protocol = filename.substr(0, 7);

    // get ZED SDK version
    int major_dll, minor_dll, patch_dll;
    getZEDSDKBuildVersion(major_dll, minor_dll, patch_dll);
    if (major_dll < 3){
        std::cerr << "SUPPORT ONLY SDK 3.*" << std::endl;
    }

    // init zed camera
    sl::InitParameters init_params;
    init_params.depth_minimum_distance = 0.5;
    init_params.depth_mode = sl::DEPTH_MODE::ULTRA;
    init_params.camera_resolution = sl::RESOLUTION::HD720;// sl::RESOLUTION::HD1080, sl::RESOLUTION::HD720
    init_params.coordinate_units = sl::UNIT::MILLIMETER;
    init_params.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP_X_FWD;  /**< Right-Handed with Z pointing up and X forward. Used in ROS (REP 103). */
    init_params.sdk_cuda_ctx = (CUcontext)detector.get_cuda_context();//ak to bude nadavat na CUDNN tak treba zakomentova/odkomentovat
    init_params.sdk_gpu_id = detector.cur_gpu_id;
    //init_params.camera_buffer_count_linux = 2;

    if (file_ext == "svo") init_params.input.setFromSVOFile(filename.c_str());

    zed.open(init_params);
    if (!zed.isOpened()) {
        std::cout << " Error: ZED Camera should be connected to USB 3.0. And ZED_SDK should be installed. \n";
        getchar();
        return;
    }

    // Check camera model
    sl::MODEL cam_model = zed.getCameraInformation().camera_model;

    zed.setCameraSettings(sl::VIDEO_SETTINGS::WHITEBALANCE_AUTO, 1);

#ifdef CAMERA_DETECTION_RECORD_VIDEO_SVO
    sl::ERROR_CODE err;
    err = zed.enableRecording(sl::RecordingParameters("myVideoFile.svo", sl::SVO_COMPRESSION_MODE::H264));
    if (err != sl::ERROR_CODE::SUCCESS) {
        std::cout <<"CAMERA_DETECTION_RECORD_VIDEO_SVO: " <<  sl::toString(err) << std::endl;
    }
#endif// CAMERA_DETECTION_RECORD_VIDEO_SVO

    if (filename.size() == 0) return;

#ifdef CAMERA_DETECTION_CARSTATE
    // Set parameters for Positional Tracking
    sl::PositionalTrackingParameters tracking_parameters;
    zed.enablePositionalTracking(tracking_parameters);
    sl::Pose camera_path;
    sl::POSITIONAL_TRACKING_STATE tracking_state;
#endif// CAMERA_DETECTION_CARSTATE

#ifdef CAMERA_DETECTION_RECORD_VIDEO
    #ifdef CV_VERSION_EPOCH // OpenCV 2.x
        output_video.open(out_videofile, CV_FOURCC('D', 'I', 'V', 'X'), 30, cv::Size(1280, 720), true);
    #else
        output_video.open(out_videofile, cv::VideoWriter::fourcc('D', 'I', 'V', 'X'), 30, cv::Size(1280, 720), true);
    #endif
#endif//CAMERA_DETECTION_RECORD_VIDEO

    while (ros::ok())
    {
        auto start = std::chrono::steady_clock::now();
        predict(detector, cam_model);
        auto finish = std::chrono::steady_clock::now();
        auto timePerFrame = std::chrono::duration_cast<std::chrono::milliseconds>(finish - start).count() / 1000.f;
        float timeDiff = TIME_PER_FRAME - timePerFrame;

        if (timeDiff > 0.f)
        {
            sleep(timeDiff);
            std::cout <<"Sleeping time: "<< timeDiff << std::endl;
        }
        else
        {
            //defenzivne programovanie ftw
        }
    }
}

void CameraConeDetection::predict(Detector &detector, sl::MODEL &cam_model) {
    std::vector<std::string> obj_names = {"yellow_cone", "blue_cone", "orange_cone_small", "orange_cone_big"};

    cv::Mat cur_frame;
    cv::Mat zed_cloud;

    std_msgs::Empty empty;

    sl::SensorsData data;

#ifdef CAMERA_DETECTION_CARSTATE
    // Set parameters for Positional Tracking
    sl::Pose camera_path;
    sl::POSITIONAL_TRACKING_STATE tracking_state;
#endif// CAMERA_DETECTION_CARSTATE

    //TODO function body
    if (zed.grab() == sl::ERROR_CODE::SUCCESS){
        cur_frame = zed_capture_rgb(zed);
        zed_cloud = zed_capture_3d(zed);
        if (cur_frame.empty() ) {
            std::cout << " exit_flag: detection_data.cap_frame.size = " << cur_frame.size() << std::endl;
            cur_frame = cv::Mat(cur_frame.size(), CV_8UC3);
        }

        //sync with LidarConeDetection
        m_signalPublisher.publish(empty);

        std::vector<bbox_t> result_vec = detector.detect(cur_frame, thresh);
        result_vec = get_3d_coordinates(result_vec, zed_cloud);

        sgtdv_msgs::ConeArr coneArr;
        sgtdv_msgs::Cone cone;
        sgtdv_msgs::Point2D point2D;
        #ifdef CAMERA_DETECTION_FAKE_LIDAR
            sgtdv_msgs::Point2DArr point2DArr;
        #endif//CAMERA_DETECTION_FAKE_LIDAR
        for (auto &i : result_vec) {
            point2D.x = i.x_3d;
            point2D.y = i.y_3d;
            cone.coords = point2D;
            #ifdef CAMERA_DETECTION_FAKE_LIDAR
                point2DArr.points.push_back(point2D);
            #endif//CAMERA_DETECTION_FAKE_LIDAR

            std::string obj_name = obj_names[i.obj_id];
            if (i.obj_id==0) //yellow_cone
                cone.color= 'y';
            if (i.obj_id==1) //blue_cone
                cone.color= 'b';
            if (i.obj_id==2) //orange_cone_small
                cone.color= 's';
            if (i.obj_id==3) //orange_cone_big
                cone.color= 'g';
            coneArr.cones.push_back(cone);
        }
        m_conePublisher.publish(coneArr);
        #ifdef CAMERA_DETECTION_FAKE_LIDAR
            m_lidarConePublisher.publish(point2DArr);
        #endif//CAMERA_DETECTION_FAKE_LIDAR

#ifdef CAMERA_DETECTION_CARSTATE
        sgtdv_msgs::CarState carState;
        sgtdv_msgs::Point2D carPoint2D;
        tracking_state = zed.getPosition(camera_path, sl::REFERENCE_FRAME::WORLD); //get actual position
        //std::cout << "Camera position: X=" << camera_path.getTranslation().x << " Y=" << camera_path.getTranslation().y << " Z=" << camera_path.getTranslation().z << std::endl;
        //std::cout << "Camera Euler rotation: X=" << camera_path.getEulerAngles().x << " Y=" << camera_path.getEulerAngles().y << " Z=" << camera_path.getEulerAngles().z << std::endl;
        carPoint2D.x = camera_path.getTranslation().x;
        carPoint2D.y = camera_path.getTranslation().y;
        carState.position = carPoint2D;
        carState.yaw = camera_path.getEulerAngles().z;

        for (size_t i = 0; i < carState.covariance.size(); i++) {
            carState.covariance[i] = camera_path.pose_covariance[i];
        }

        m_carStatePublisher.publish(carState);
#endif//CAMERA_DETECTION_CARSTATE

        if (cam_model == sl::MODEL::ZED2) {
            if (zed.getSensorsData(data, sl::TIME_REFERENCE::CURRENT) == sl::ERROR_CODE::SUCCESS) {
                // Filtered orientation quaternion
                std::cout << "IMU Orientation x: " << data.imu.pose.getOrientation().ox << "y: " << data.imu.pose.getOrientation().oy <<
                "z: " << data.imu.pose.getOrientation().oz << "w: " << data.imu.pose.getOrientation().ow << std::endl;

                // Filtered acceleration
                std::cout << "IMU Acceleration [m/sec^2] x: " << data.imu.linear_acceleration.x << "y: " << data.imu.linear_acceleration.y <<
                            "z: " << data.imu.linear_acceleration.z << std::endl;

                // Filtered angular velocities
                std::cout << "IMU angular velocities [deg/sec] x: " << data.imu.angular_velocity.x << "y: " << data.imu.angular_velocity.y <<
                            "z: " << data.imu.angular_velocity.z << std::endl;
            }
        }


#ifdef CAMERA_DETECTION_CAMERA_SHOW
        draw_boxes(cur_frame, result_vec, obj_names);
#endif //CAMERA_DETECTION_CAMERA_SHOW
#ifdef CAMERA_DETECTION_RECORD_VIDEO
        //video.write(cur_frame);
        output_video << cur_frame;
        //video.write(draw_boxes(cur_frame, result_vec, obj_names));
#endif//CAMERA_DETECTION_RECORD_VIDEO
#ifdef CAMERA_DETECTION_CONSOLE_SHOW
        show_console_result(result_vec, obj_names);
#endif //CAMERA_DETECTION_CONSOLE_SHOW
     }
}

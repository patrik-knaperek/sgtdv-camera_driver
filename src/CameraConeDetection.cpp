/*****************************************************/
//Organization: Stuba Green Team
//Authors: Matúš Tomšík, Juraj Krasňanský
/*****************************************************/


#include "../include/CameraConeDetection.h"

//#define CARSTATE
//#define CAMERA_SHOW
//#define CONSOLE_SHOW

CameraConeDetection::CameraConeDetection()
{

}

CameraConeDetection::~CameraConeDetection()
{
    if (zed.isOpened()) {
        zed.close();
        std::cout << "zed camera closed" << std::endl;
    }
}

void CameraConeDetection::SetSignalPublisher(ros::Publisher signalPublisher)
{
    m_signalPublisher = signalPublisher;
}

void CameraConeDetection::SetConePublisher(ros::Publisher conePublisher)
{
    m_conePublisher = conePublisher;
}

void CameraConeDetection::SetcarStatePublisher(ros::Publisher carStatePublisher)
{
    m_carStatePublisher = carStatePublisher;
}
#ifdef CAMERA_SHOW
void CameraConeDetection::draw_boxes(cv::Mat mat_img, std::vector<bbox_t> result_vec, std::vector<std::string> obj_names, int current_det_fps = -1, int current_cap_fps = -1) {
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
}
#endif //CAMERA_SHOW

std::vector<std::string> CameraConeDetection::objects_names_from_file(std::string const filename) {
	std::ifstream file(filename);
	std::vector<std::string> file_lines;
	if (!file.is_open()) return file_lines;
	for(std::string line; file >> line;) file_lines.push_back(line);
	std::cout << "object names loaded \n";
	return file_lines;
}

#ifdef CONSOLE_SHOW
    void CameraConeDetection::show_console_result(std::vector<bbox_t> const result_vec, std::vector<std::string> const obj_names, int frame_id = -1) {
        if (frame_id >= 0) std::cout << " Frame: " << frame_id << std::endl;
        for (auto &i : result_vec) {
            if (obj_names.size() > i.obj_id) std::cout << obj_names[i.obj_id] << " - ";
            std::string obj_name = obj_names[i.obj_id];
            std::cout << "obj_name = " << obj_name << ",  x = " << i.x_3d << ", y = " << i.y_3d
                << std::setprecision(3) << ", prob = " << i.prob << std::endl;
        }
    }
#endif //CONSOLE_SHOW
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
    case sl::MAT_TYPE_32F_C1:
        cv_type = CV_32FC1;
        break;
    case sl::MAT_TYPE_32F_C2:
        cv_type = CV_32FC2;
        break;
    case sl::MAT_TYPE_32F_C3:
        cv_type = CV_32FC3;
        break;
    case sl::MAT_TYPE_32F_C4:
        cv_type = CV_32FC4;
        break;
    case sl::MAT_TYPE_8U_C1:
        cv_type = CV_8UC1;
        break;
    case sl::MAT_TYPE_8U_C2:
        cv_type = CV_8UC2;
        break;
    case sl::MAT_TYPE_8U_C3:
        cv_type = CV_8UC3;
        break;
    case sl::MAT_TYPE_8U_C4:
        cv_type = CV_8UC4;
        break;
    default:
        break;
    }
    return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(sl::MEM_CPU));
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
    zed.retrieveMeasure(cur_cloud, sl::MEASURE_XYZ);
    return slMat2cvMat(cur_cloud).clone();
}


void CameraConeDetection::Do()
{
    Detector detector(cfg_file, weights_file); //Darknet
    auto obj_names = objects_names_from_file(names_file);

    std::string const file_ext = filename.substr(filename.find_last_of(".") + 1);
    std::string const protocol = filename.substr(0, 7);

    // init zed camera
    sl::InitParameters init_params;
    init_params.depth_minimum_distance = 0.5;
    init_params.depth_mode = sl::DEPTH_MODE_ULTRA;
    init_params.camera_resolution = sl::RESOLUTION_HD720;// sl::RESOLUTION_HD1080, sl::RESOLUTION_HD720
    init_params.coordinate_units = sl::UNIT_MILLIMETER;
    init_params.coordinate_system = sl::COORDINATE_SYSTEM_RIGHT_HANDED_Z_UP;
    init_params.sdk_cuda_ctx = (CUcontext)detector.get_cuda_context();//ak to bude nadavat na CUDNN tak treba zakomentova/odkomentovat
    init_params.sdk_gpu_id = detector.cur_gpu_id;
    init_params.camera_buffer_count_linux = 2;

    if (file_ext == "svo") init_params.svo_input_filename.set(filename.c_str());

    zed.open(init_params);
    if (!zed.isOpened()) {
        std::cout << " Error: ZED Camera should be connected to USB 3.0. And ZED_SDK should be installed. \n";
        getchar();
        return;
    }

    if (filename.size() == 0) return;

    cv::Mat cur_frame;
    cv::Mat zed_cloud;

    std_msgs::Empty empty;

#ifdef CARSTATE
    // Set parameters for Positional Tracking
    zed.enableTracking();
    sl::Pose camera_path;
    sl::TRACKING_STATE tracking_state;
#endif// CARSTATE

    while (ros::ok())
    {
        auto start = std::chrono::steady_clock::now();

        //TODO function body
        if (zed.grab() == sl::SUCCESS){
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
            for (auto &i : result_vec) {
                point2D.x = i.x_3d;
                point2D.y = i.y_3d;
                cone.coords = point2D;

                std::string obj_name = obj_names[i.obj_id];
                if (i.obj_id==0)
                    cone.color= 'y';
                if (i.obj_id==1)
                    cone.color= 'b';
                coneArr.cones.push_back(cone);
            }
            m_conePublisher.publish(coneArr);

#ifdef CARSTATE
            sgtdv_msgs::CarState carState;
            sgtdv_msgs::Point2D carPoint2D;
            tracking_state = zed.getPosition(camera_path, sl::REFERENCE_FRAME_WORLD); //get actual position
            //std::cout << "Camera position: X=" << camera_path.getTranslation().x << " Y=" << camera_path.getTranslation().y << " Z=" << camera_path.getTranslation().z << std::endl;
            //std::cout << "Camera Euler rotation: X=" << camera_path.getEulerAngles().x << " Y=" << camera_path.getEulerAngles().y << " Z=" << camera_path.getEulerAngles().z << std::endl;
            carPoint2D.x = camera_path.getTranslation().x;
            carPoint2D.y = camera_path.getTranslation().y;
            carState.position = carPoint2D;
            carState.yaw = camera_path.getEulerAngles().z;

            m_carStatePublisher.publish(carState);
#endif//CARSTATE


#ifdef CAMERA_SHOW
            draw_boxes(cur_frame, result_vec, obj_names);
#endif //CAMERA_SHOW
#ifdef CONSOLE_SHOW
            show_console_result(result_vec, obj_names);
#endif //CONSOLE_SHOW
         }
        /******************************************************************/

        auto finish = std::chrono::steady_clock::now();
        auto timePerFrame = std::chrono::duration_cast<std::chrono::milliseconds>(finish - start).count() / 1000.f;
        float timeDiff = TIME_PER_FRAME - timePerFrame;

        if (timeDiff > 0.f)
        {
            sleep(timeDiff);
        }
        else
        {
            //defenzivne programovanie ftw
        }
    }
}

#include "../include/CameraConeDetection.h"

#define CARSTATE

CameraConeDetection::CameraConeDetection()
{

}

CameraConeDetection::~CameraConeDetection()
{

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

void CameraConeDetection::Do()
{
    while (ros::ok())
    {
        auto start = std::chrono::steady_clock::now();

        //TODO function body
        predictWithVideo();

        auto finish = std::chrono::steady_clock::now();
        float timeDiff = TIME_PER_FRAME - std::chrono::duration_cast<std::chrono::milliseconds>(finish - start).count() / 1000.f;

        if (timeDiff > 0.f)
        {
            sleep(timeDiff);
        }
        else
        {
            //defenzivne programovanie ftw
            break;
        }        
    }
}

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

void CameraConeDetection::draw_boxes(cv::Mat mat_img, std::vector<bbox_t> result_vec, std::vector<std::string> obj_names, int current_det_fps = -1, int current_cap_fps = -1) {
    int const colors[6][3] = { { 1,0,1 },{ 0,0,1 },{ 0,1,1 },{ 0,1,0 },{ 1,1,0 },{ 1,0,0 } };

    for (auto &i : result_vec) {
        cv::Scalar color = { 255,0,0 };//obj_id_to_color(i.obj_id);
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
                ss << std::fixed << std::setprecision(2) << "x:" << i.x_3d << "m y:" << i.y_3d << "m z:" << i.z_3d << "m ";
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
}


void CameraConeDetection::show_console_result(std::vector<bbox_t> const result_vec, std::vector<std::string> const obj_names, int frame_id = -1) {
    if (frame_id >= 0) std::cout << " Frame: " << frame_id << std::endl;
    for (auto &i : result_vec) {
        if (obj_names.size() > i.obj_id) std::cout << obj_names[i.obj_id] << " - ";
        std::cout << "obj_id = " << i.obj_id << ",  x = " << i.x << ", y = " << i.y
            << ", w = " << i.w << ", h = " << i.h
            << std::setprecision(3) << ", prob = " << i.prob << std::endl;
    }
}

std::vector<std::string> CameraConeDetection::objects_names_from_file(std::string const filename) {
    std::ifstream file(filename);
    std::vector<std::string> file_lines;
    if (!file.is_open()) return file_lines;
    for(std::string line; getline(file, line);) file_lines.push_back(line);
    std::cout << "object names loaded \n";
    return file_lines;
}

int CameraConeDetection::predictWithVideo() {
    static sl::Camera zed; // ZED-camera

    Detector detector(cfg_file, weights_file);
    auto obj_names = objects_names_from_file(names_file);

    bool const send_network = false;        // true - for remote detection

    bool detection_sync = true;
    while (true)
        {
            std::cout << "input image or video filename: ";
            if(filename.size() == 0) std::cin >> filename;
            if (filename.size() == 0) break;

            bool show_small_boxes = false;
            std::string const file_ext = filename.substr(filename.find_last_of(".") + 1);
            std::string const protocol = filename.substr(0, 7);
            if (file_ext == "avi" || file_ext == "mp4" || filename == "zed_camera" || file_ext == "svo" || filename == "web_camera")   // ZED stereo camera
            {
                if (filename == "zed_camera" || filename == "web_camera")
                    detection_sync = false;

                cv::Mat cur_frame;
                std::atomic<int> fps_cap_counter(0), fps_det_counter(0);
                std::atomic<int> current_fps_cap(0), current_fps_det(0);
                std::atomic<bool> exit_flag(false);
                std::chrono::steady_clock::time_point steady_start, steady_end;
                int video_fps = 25;
                bool use_zed_camera = false;

                #ifdef ZED_STEREO
                    sl::InitParameters init_params;
                    init_params.depth_minimum_distance = 0.5;
                    init_params.depth_mode = sl::DEPTH_MODE_ULTRA;
                    init_params.camera_resolution = sl::RESOLUTION_HD720;// sl::RESOLUTION_HD1080, sl::RESOLUTION_HD720
                    init_params.coordinate_units = sl::UNIT_MILLIMETER;
                    init_params.coordinate_system = sl::COORDINATE_SYSTEM_RIGHT_HANDED_Z_UP;
                    //init_params.sdk_cuda_ctx = (CUcontext)detector.get_cuda_context();
                    init_params.sdk_gpu_id = detector.cur_gpu_id;
                    init_params.camera_buffer_count_linux = 2;
                    if (file_ext == "svo") init_params.svo_input_filename.set(filename.c_str());
                    if (filename == "zed_camera" || file_ext == "svo") {
                        std::cout << "ZED 3D Camera " << zed.open(init_params) << std::endl;
                        if (!zed.isOpened()) {
                            std::cout << " Error: ZED Camera should be connected to USB 3.0. And ZED_SDK should be installed. \n";
                            getchar();
                            return 0;
                        }
                        cur_frame = zed_capture_rgb(zed);
                        use_zed_camera = true;
                    }
                    cv::VideoCapture cap;
                    if (filename == "web_camera") {
                        cap.open(0);
                        cap >> cur_frame;
                    } else if (!use_zed_camera) {
                        cap.open(filename);
                        cap >> cur_frame;
                    }
                    #ifdef CV_VERSION_EPOCH // OpenCV 2.x
                        video_fps = cap.get(CV_CAP_PROP_FPS);
                    #else
                        video_fps = cap.get(cv::CAP_PROP_FPS);
                    #endif
                        cv::Size const frame_size = cur_frame.size();
                        //cv::Size const frame_size(cap.get(CV_CAP_PROP_FRAME_WIDTH), cap.get(CV_CAP_PROP_FRAME_HEIGHT));
                        std::cout << "\n Video size: " << frame_size << std::endl;
                #endif  // ZED_STEREO

                #ifdef CARSTATE
                    // Set parameters for Positional Tracking
                    zed.enableTracking();
                    sl::Pose camera_path;
                    sl::TRACKING_STATE tracking_state;
                #endif   // CARSTATE

                struct detection_data_t {
                    cv::Mat cap_frame;
                    std::shared_ptr<image_t> det_image;
                    std::vector<bbox_t> result_vec;
                    cv::Mat draw_frame;
                    bool new_detection;
                    uint64_t frame_id;
                    bool exit_flag;
                    cv::Mat zed_cloud;
                    std::queue<cv::Mat> track_optflow_queue;
                    detection_data_t() : exit_flag(false), new_detection(false) {}
                };

                const bool sync = detection_sync; // sync data exchange
                send_one_replaceable_object_t<detection_data_t> cap2prepare(sync), cap2draw(sync),
                                    prepare2detect(sync), detect2draw(sync), draw2show(sync), draw2write(sync), draw2net(sync);

                std::thread t_cap, t_prepare, t_detect, t_post, t_draw, t_write, t_network;

                // capture new video-frame
                if (t_cap.joinable()) t_cap.join();
                t_cap = std::thread([&]()
                {
                    uint64_t frame_id = 0;
                    detection_data_t detection_data;
                    do {
                        detection_data = detection_data_t();
#ifdef ZED_STEREO
                        if (use_zed_camera) {
                            while (zed.grab() != sl::SUCCESS) std::this_thread::sleep_for(std::chrono::milliseconds(2));
                            detection_data.cap_frame = zed_capture_rgb(zed);
                            detection_data.zed_cloud = zed_capture_3d(zed);
                            #ifdef CARSTATE
                                sgtdv_msgs::CarState carState;
                                sgtdv_msgs::Point2D point2D;
                                tracking_state = zed.getPosition(camera_path, sl::REFERENCE_FRAME_WORLD);
                                // Display translation
                                //std::cout << "Camera position: X=" << camera_path.getTranslation().x << " Y=" << camera_path.getTranslation().y << " Z=" << camera_path.getTranslation().z << std::endl;
                                // Display orientation quaternion
                                //std::cout << "Camera Euler rotation: X=" << camera_path.getEulerAngles().x << " Y=" << camera_path.getEulerAngles().y << " Z=" << camera_path.getEulerAngles().z << std::endl;
                                point2D.x = camera_path.getTranslation().x;
                                point2D.y = camera_path.getTranslation().y;
                                carState.position = point2D;
                                carState.yaw = camera_path.getEulerAngles().z;

                                m_carStatePublisher.publish(carState);

                            #endif   // CARSTATE
    }
                        else
#endif   // ZED_STEREO
                        {
                            cap >> detection_data.cap_frame;
                        }
                        fps_cap_counter++;
                        detection_data.frame_id = frame_id++;
                        if (detection_data.cap_frame.empty() || exit_flag) {
                            std::cout << " exit_flag: detection_data.cap_frame.size = " << detection_data.cap_frame.size() << std::endl;
                            detection_data.exit_flag = true;
                            detection_data.cap_frame = cv::Mat(frame_size, CV_8UC3);
                        }

                        if (!detection_sync) {
                            cap2draw.send(detection_data);       // skip detection
                        }
                        cap2prepare.send(detection_data);
                    } while (!detection_data.exit_flag);
                    std::cout << " t_cap exit \n";
                });


                // pre-processing video frame (resize, convertion)
                t_prepare = std::thread([&]()
                {
                    std::shared_ptr<image_t> det_image;
                    detection_data_t detection_data;
                    do {
                        detection_data = cap2prepare.receive();

                        det_image = detector.mat_to_image_resize(detection_data.cap_frame);
                        detection_data.det_image = det_image;
                        prepare2detect.send(detection_data);    // detection

                    } while (!detection_data.exit_flag);
                    std::cout << " t_prepare exit \n";
                });


                // detection by Yolo
                if (t_detect.joinable()) t_detect.join();
                t_detect = std::thread([&]()
                {
                    std::shared_ptr<image_t> det_image;
                    detection_data_t detection_data;
                    do {
                        detection_data = prepare2detect.receive();
                        det_image = detection_data.det_image;
                        std::vector<bbox_t> result_vec;

                        if(det_image) {
                            std_msgs::Empty empty;
                            m_signalPublisher.publish(empty);

                            result_vec = detector.detect_resized(*det_image, frame_size.width, frame_size.height, thresh, true);  // true
                        }
                        fps_det_counter++;
                        //std::this_thread::sleep_for(std::chrono::milliseconds(150));

                        detection_data.new_detection = true;
                        detection_data.result_vec = result_vec;
                        detect2draw.send(detection_data);
                    } while (!detection_data.exit_flag);
                    std::cout << " t_detect exit \n";
                });
                // draw rectangles (and track objects)
                t_draw = std::thread([&]()
                {
                    std::queue<cv::Mat> track_optflow_queue;
                    detection_data_t detection_data;
                    do {

                        // for Video-file
                        if (detection_sync) {
                            detection_data = detect2draw.receive();
                        }
                        // for Video-camera
                        else
                        {
                            // get new Detection result if present
                            if (detect2draw.is_object_present()) {
                                cv::Mat old_cap_frame = detection_data.cap_frame;   // use old captured frame
                                detection_data = detect2draw.receive();
                                if (!old_cap_frame.empty()) detection_data.cap_frame = old_cap_frame;
                            }
                            // get new Captured frame
                            else {
                                std::vector<bbox_t> old_result_vec = detection_data.result_vec; // use old detections
                                detection_data = cap2draw.receive();
                                detection_data.result_vec = old_result_vec;
                            }
                        }

                        cv::Mat cap_frame = detection_data.cap_frame;
                        cv::Mat draw_frame = detection_data.cap_frame.clone();
                        std::vector<bbox_t> result_vec = detection_data.result_vec;

                        int frame_story = std::max(5, current_fps_cap.load());
                        result_vec = detector.tracking_id(result_vec, true, frame_story, 40);

                        if (use_zed_camera && !detection_data.zed_cloud.empty()) {
                            result_vec = get_3d_coordinates(result_vec, detection_data.zed_cloud);
                        }

                        draw_boxes(draw_frame, result_vec, obj_names, current_fps_det, current_fps_cap);
                        //show_console_result(result_vec, obj_names, detection_data.frame_id);
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
                            //std::cout << "obj_name" << obj_name << "x = " << i.x_3d << " y = " << i.y_3d << std::endl;
                        }

                        m_conePublisher.publish(coneArr);


                        detection_data.result_vec = result_vec;
                        detection_data.draw_frame = draw_frame;
                        draw2show.send(detection_data);
                        if (send_network) draw2net.send(detection_data);
                    } while (!detection_data.exit_flag);
                    std::cout << " t_draw exit \n";
                });
                // send detection to the network
                t_network = std::thread([&]()
                {
                    if (send_network) {
                        detection_data_t detection_data;
                        do {
                            detection_data = draw2net.receive();

                            detector.send_json_http(detection_data.result_vec, obj_names, detection_data.frame_id, filename);

                        } while (!detection_data.exit_flag);
                    }
                    std::cout << " t_network exit \n";
                });


                // show detection
                detection_data_t detection_data;
                do {

                    steady_end = std::chrono::steady_clock::now();
                    float time_sec = std::chrono::duration<double>(steady_end - steady_start).count();
                    if (time_sec >= 1) {
                        current_fps_det = fps_det_counter.load() / time_sec;
                        current_fps_cap = fps_cap_counter.load() / time_sec;
                        steady_start = steady_end;
                        fps_det_counter = 0;
                        fps_cap_counter = 0;
                    }

                    detection_data = draw2show.receive();
                    cv::Mat draw_frame = detection_data.draw_frame;

                    //if (extrapolate_flag) {
                    //    cv::putText(draw_frame, "extrapolate", cv::Point2f(10, 40), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0, cv::Scalar(50, 50, 0), 2);
                    //}

                    cv::imshow("window name", draw_frame);
                    int key = cv::waitKey(3);    // 3 or 16ms
                    if (key == 'f') show_small_boxes = !show_small_boxes;
                    if (key == 'p') while (true) if (cv::waitKey(100) == 'p') break;
                    //if (key == 'e') extrapolate_flag = !extrapolate_flag;
                    if (key == 27) { exit_flag = true;}

                    //std::cout << " current_fps_det = " << current_fps_det << ", current_fps_cap = " << current_fps_cap << std::endl;
                } while (!detection_data.exit_flag);
                std::cout << " show detection exit \n";

                cv::destroyWindow("window name");
                // wait for all threads
                if (t_cap.joinable()) t_cap.join();
                if (t_prepare.joinable()) t_prepare.join();
                if (t_detect.joinable()) t_detect.join();
                if (t_post.joinable()) t_post.join();
                if (t_draw.joinable()) t_draw.join();
                if (t_write.joinable()) t_write.join();
                if (t_network.joinable()) t_network.join();

                break;
            }
        filename.clear();
    }
}

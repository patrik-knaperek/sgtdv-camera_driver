/*****************************************************/
//Organization: Stuba Green Team
//Authors: Matúš Tomšík, Juraj Krasňanský, Patrik Knaperek
/*****************************************************/

#include <ros/ros.h>

#include "camera_cone_detection.h"

CameraConeDetection::CameraConeDetection(const Params& nn_params)
: detector_(nn_params.cfg_file, nn_params.weights_file)
{

}

std::vector <bbox_t> CameraConeDetection::detect(const cv::Mat& cur_frame, const cv::Mat& xyzrgba)
{
  std::vector <bbox_t> result_vec = detector_.detect(cur_frame);
  get3dCoordinates(&result_vec, xyzrgba);
  
  return result_vec;
}

void CameraConeDetection::get3dCoordinates(std::vector <bbox_t>* bbox_vect, const cv::Mat& xyzrgba) const
{
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
          const int i = center_i + x;
          const int j = center_j + y;
          const auto elem = xyzrgba.at<cv::Vec4f>(j, i);  // x,y,z,w
          
          if (i >= 0 && i < xyzrgba.cols && j >= 0 && j < xyzrgba.rows
              && std::isfinite(elem[2])) // z-coordinate -> valid measure
          {
            if (std::isfinite(elem[2])) // z-coordinate -> valid measure
            {
              x_vect.push_back(elem[0]);
              y_vect.push_back(elem[1]);
              z_vect.push_back(elem[2]);
            }
            
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

float CameraConeDetection::getMedian(std::vector<float> &v) const
{
  const size_t n = v.size() / 2;
  std::nth_element(v.begin(), v.begin() + n, v.end());
  return v[n];
}

void CameraConeDetection::drawBoxes(cv::Mat* mat_img, const std::vector <bbox_t>& result_vec,
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

  cv::waitKey(3);
}

void CameraConeDetection::getObjectsNamesFromFile(const std::string& names_file)
{
	std::ifstream file(names_file);
	if (!file.is_open())
  {
    ROS_ERROR_STREAM("Could not open file " << names_file);
    ros::shutdown();
    return;
  }

	for(std::string line; file >> line;)
    obj_names_.push_back(line);

  if (obj_names_.size() == 0)
  {
    ROS_ERROR("Unable to obtain object names.");
    ros::shutdown();
    return;
  }
  else
  {
    ROS_INFO("object names loaded");	
  }
}

void CameraConeDetection::showConsoleResult(const std::vector<bbox_t>& result_vec) const
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
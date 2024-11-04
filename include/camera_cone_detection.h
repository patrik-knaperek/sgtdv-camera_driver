/*****************************************************/
//Organization: Stuba Green Team
//Authors: Matúš Tomšík, Juraj Krasňanský, Patrik Knaperek
/*****************************************************/

#include "yolo_v2_class.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/core/version.hpp>
#include <opencv2/highgui/highgui_c.h>

class CameraConeDetection
{
public:
  enum CONE_CLASSES
  {
    YELLOW,
    BLUE,
    ORANGE_SMALL,
    ORANGE_BIG
  };

  struct Params
  {
    std::string cfg_file;
    std::string weights_file;
  };

public:
  explicit CameraConeDetection(const Params& nn_params);
  ~CameraConeDetection() = default;

  std::vector <bbox_t> detect(const cv::Mat& cur_frame, const cv::Mat& xyzrgba);
  
  void drawBoxes(cv::Mat* mat_img, const std::vector <bbox_t>& result_vec,
                const int current_det_fps = -1, const int current_cap_fps = -1) const;

  float getMedian(std::vector<float> &v) const;

  void get3dCoordinates(std::vector <bbox_t>* bbox_vect, const cv::Mat& xyzrgba) const;

  void getObjectsNamesFromFile(const std::string& names_file);

  void showConsoleResult(const std::vector <bbox_t>& result_vec) const;

  LIB_API void* getDetectorCudaContext(void)
  {
    return detector_.get_cuda_context();
  };

  int getGPUid(void)
  {
    return detector_.cur_gpu_id;
  }

private:

  static constexpr float DETECT_TH = 0.2;

  Detector detector_; // Darknet NN object
  std::vector<std::string> obj_names_;
};

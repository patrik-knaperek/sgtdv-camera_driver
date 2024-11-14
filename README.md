# **Camera Driver package**

___

&copy; **SGT Driverless**

**Authors:** Juraj Krasňanský, Matúš Tomšík, Viktor Budylin, Alexander Bobrov, Patrik Knaperek

**Objective:** Cone detection, position estimation and classification from ZED camera picture; visual odometry and IMU data interface.

___

### Related packages
* `visual_odometry`

### Requirements

* [**CUDA 10.0**](https://developer.nvidia.com/embedded/jetpack)
* [**OpenCV >= 4.0.1**](https://developer.nvidia.com/embedded/jetpack)
* [**ZED SDK 3.X**](https://www.stereolabs.com/developers/release/)
* [**Darknet**](https://github.com/AlexeyAB/darknet)  
  

### Darknet compilation
* Set in `Makefile`:
  * GPU=1
  * CUDNN=1
  * CUDNN_HALF=1
  * LIBSO=1
* Then compile with `$ make`
* Copy generated `libdarknet.so` file into `camera_driver/include`


## Compilation

Configuration files for NN, generated *.weights and *.svo files are stored in folder [**Darknet_cone_detection**](https://drive.google.com/drive/folders/144MJlPqqrMii9dVJtaWv_vCwrJNkGFed?usp=sharing) on G-Drive. Copy this folder into `src/camera_driver/`. If any file is changed, it needs to be updated on G-Drive. New weights also lie on [**G-Drive**](https://drive.google.com/drive/u/1/folders/1LW0ZmNBE1v93NTcOAzq3gNRCSCzKWU6H).

The following packages have to be built at first:
  - `sgtdv_msgs`

In folder `ros_implementation/src/` run:
```sh
$ catkin build camera_driver -DCMAKE_BUILD_TYPE=Release
```

Set the input stream in `CameraConeDetection.h` with variable `filename` value:
 * **"zed_camera"** : live camera picture
 * **"<path_to_.svo_file>"** : recorded SVO video

## Launch
```sh
$ source ros_implementation/devel/setup.bash
$ roslaunch camera_driver camera_driver.launch
```

### Launch configuration
* `param/camera_driver.yaml`
  - `obj_names_filename` : file name with stored names of classes
  - `cfg_filename` : NN configuration file name
  - `weights_filename` : weights configuration file name
  - `output_video_filename` : filename of recorded MP4 video
  - `output_svo_filename` : filename of recorded SVO video
  - `input_stream` : define input source - "zed" or relative path to a SVO video
  - `publish_carstate` : publish camera pose (left eye by default) from ZED visual odometry in `odom` frame to `/camera_pose` topic
  - `camera_show` : show live camera stream with bounding boxes in window - not possible through SSH
  - `console_show` : print out cone detection results into console
  - `record_video` : record MP4 video from camera stream
  - `record_video_svo` : record SVO video from camera stream - can be used as offline camera input

### RViz visualization
In new terminal run:
```sh
$ source ros_implementation/devel/setup.bash
$ roslaunch data_visualization data_visualization_camera.launch
```

 ## Visual odometry
 Node `visual_odometry` subscribes `/camera_pose` topic from `camera_driver` node and publishes transformation from `base_link` to `odom` frame on general `/tf` topic.

## Benchmarking different YOLO configurations
It is claimed by Stephane Charette that yolov4-tiny is both faster to train and more accurate, has faster inference time than yolov7-tiny. Though we were unable to train yolov4-tiny to be more precise than yolov7-tiny. It may be caused by the overall complexity of FSOCO dataset. To get better runtime performance (inference time) we chose to decrease network size. In case there are any questions, please consult Darknet [FAQ](https://www.ccoderun.ca/programming/darknet_faq/#fps). All of the tests were run on Jetson AGX Xavier. yolov4-tiny-obj turned out to be even slower than FSOCO yolov4-tiny. yolov4-tiny-3l-obj had visible problems with detection seen on visualization. Never use those network configurations.

|Model and dataset|Resolution|FPS w/o viz|FPS with viz|mAP@0.5|mAP@0.75|
|-----------------|----------|-----------|------------|-------|--------|
|FSOCO yolov7-tiny| 640x640  |  21.3     |    18      |  76%  |   54%  |
|FSOCO yolov7-tiny| 416x416  |  23.8     |  not tested|not tested|not tested|
|FSOCO yolov4-tiny| 608x608  |  23.26|  20  | 68%  | 49%  |
|FSOCO yolov4-tiny| 416x416  |  29.5| 25  | not tested|not tested|
|own dataset yolov3-tiny| 416x416  | 30 | 19| 12%!!! | not tested|
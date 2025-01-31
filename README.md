# **camera_driver package**

___

&copy; **SGT Driverless**

**Authors:** Juraj Krasňanský, Matúš Tomšík, Viktor Budylin, Alexander Bobrov, Patrik Knaperek

**Objective:** Cone detection, position estimation and classification from ZED camera picture; visual odometry and IMU data interface.

___

## Overview

The `camera_driver` node provides an interface between [ZED SDK API](https://www.stereolabs.com/docs) and SGT-DV ROS network, producing up to 3 types of data: 
  1. The camera picture is used as input for [Darknet neural network](https://github.com/AlexeyAB/darknet), trained for detection and classification of cones specified by FS Driverless rules (currently trained on [FSOCO dataset](https://www.fsoco-dataset.com/)), employing the YOLO(v4) method.
  2. Based on visual odometry data provided by the SDK, camera pose is published.
  3. Data from built-in IMU sensor is published.

### ROS Interface

**Published topics:**
* `/camera/cones` [[`sgtdv_msgs/ConeStampedArr`](../sgtdv_msgs/msg/ConeStampedArr.msg)] : detected cones position in `camera_left` frame and color
* `/camera/pose` [[`geometry_msgs/PoseWithCovarianceStamped`](/opt/ros/noetic/share/geometry_msgs/msg/PoseWithCovarianceStamped.msg)] : position and rotation of `base_link` frame (transformed from `camera_left` frame) in `odom` frame
* `/camera/imu` [[`sensor_msgs/Imu`](/opt/ros/noetic/share/sensor_msgs/msg/Imu.msg)] : data from built-in IMU sensor in `camera_center` frame  

*If `SGT_DEBUG_STATE` macro enabled*
* `/camera_driver/debug_state` [[`sgtdv_msgs/DebugState](../sgtdv_msgs/msg/DebugState.msg)] : node lifecycle information (active/inactive, number of detected cones)

**Advertised services**
* `/camera/reset_odometry` [[`std_srvs/Empty`](/opt/ros/noetic/share/std_srvs/srv/Empty.srv)] : resets the positional tracking matrix on callback

**Parameters**
* `/obj_names_filename` : file name with stored names of classes
* `/cfg_filename` : NN configuration file name
* `/weights_filename` : weights configuration file name
* `/output_video_filename` : filename of recorded MP4 video
* `/output_svo_filename` : filename of recorded SVO video
* `/input_stream` : define input source - "zed" or relative path to a SVO video
* `/publish_carstate` : publish camera pose (left eye by default) from ZED visual odometry in `odom` frame to `/camera_pose` topic
* `/camera_show` : show live camera stream with bounding boxes in window - not possible through SSH
* `/console_show` : print out cone detection results into console
* `/record_video` : record MP4 video from camera stream
* `/record_video_svo` : record SVO video from camera stream - can be used as offline camera input


### Related packages
* [`fusion`](../fusion/README.md) - `/camera/cones` subscriber
* [`robot_localization`](../robot_localization/README.md) : `/camera/pose` and `/camera/imu` subscriber
* [`odometry_interface`](../odometry_interface/README.md) : `/camera/reset_odometry` caller and (optional) `/camera/pose` subscriber
* [`visual_odometry`](../visual_odometry/README.md) : (optional, alternating `robot_localization`) `/camera/pose` subscriber

## Installation

### Requirements

* [**CUDA**](https://developer.nvidia.com/embedded/jetpack)
* [**OpenCV >= 4.0.1**](https://developer.nvidia.com/embedded/jetpack)
* [**ZED SDK**](https://www.stereolabs.com/developers/release/) (pick correct version according to the current hardware configuration)
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

Configuration files for NN, generated `*.weights` and `*.svo` files are stored in folder [**Darknet_cone_detection**](./Darknet_cone_detection/). If a new configuration is trained, it has to be added there.

* standalone
```sh
$ catkin build camera_driver -DCMAKE_BUILD_TYPE=Release
```
* RC car setup
```sh
$ source ${SGT_ROOT}/scripts/build_rc.sh
```

### Compilation configuration
* [`SGT_Macros.h`](../SGT_Macros.h) :
  - `SGT_DEBUG_STATE` : publish node lifecycle information
  
## Launch
* standalone
```sh
$ roslaunch camera_driver camera_driver.launch
```
* perception setup
```sh
$ roslaunch fusion fusion_rc.launch
```
* camera pose tracking
```sh
$ roslaunch camera_driver camera_pose_tracking.launch
```


### Launch configuration
* [`camera_driver.yaml`](./params/camera_driver.yaml)

### RViz visualization
* standalone
```sh
$ roslaunch data_visualization data_visualization_camera.launch
```
* RC car setup
```sh
$ roslaunch data_visualization data_visualization_rc.launch
```

## Diagrams and flowcharts

<figcaption align = "center">Camera Cone Detection flowchart</figcaption>
<p align="center">
    <img src="./doc/SW flowcharts-Camera Cone Detection.svg" height="300">
</p>

## Benchmarking different YOLO configurations
It is claimed by Stephane Charette that yolov4-tiny is both faster to train and more accurate, has faster inference time than yolov7-tiny. Though we were unable to train yolov4-tiny to be more precise than yolov7-tiny. It may be caused by the overall complexity of FSOCO dataset. To get better runtime performance (inference time) we chose to decrease network size. In case there are any questions, please consult Darknet [FAQ](https://www.ccoderun.ca/programming/darknet_faq/#fps). All of the tests were run on Jetson AGX Xavier. yolov4-tiny-obj turned out to be even slower than FSOCO yolov4-tiny. yolov4-tiny-3l-obj had visible problems with detection seen on visualization. Never use those network configurations.

|Model and dataset|Resolution|FPS w/o viz|FPS with viz|mAP@0.5|mAP@0.75|
|-----------------|----------|-----------|------------|-------|--------|
|FSOCO yolov7-tiny| 640x640  |  21.3     |    18      |  76%  |   54%  |
|FSOCO yolov7-tiny| 416x416  |  23.8     |  not tested|not tested|not tested|
|FSOCO yolov4-tiny| 608x608  |  23.26|  20  | 68%  | 49%  |
|FSOCO yolov4-tiny| 416x416  |  29.5| 25  | not tested|not tested|
|own dataset yolov3-tiny| 416x416  | 30 | 19| 12%!!! | not tested|

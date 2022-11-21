# **CameraConeDetection package**

___

&copy; **SGT Driverless**

**Authors:** Juraj Krasňanský, Matúš Tomšík

**Objective:** Cone detection, position estimation and classification from ZED camera picture.

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
* Copy generated `libdarknet.so` file into `camera_cone_detection/include`


## Compilation

Configuration files for NN, generated *.weights and *.svo files are stored in folder [**Darknet_cone_detection**](https://drive.google.com/drive/folders/144MJlPqqrMii9dVJtaWv_vCwrJNkGFed?usp=sharing) on G-Drive. Copy this folder into `src/camera_cone_detection/`. If any file is changed, it needs to be updated on G-Drive.

The following packages have to be built at first:
  - `sgtdv_msgs`

In folder `ros_implementation/src/` run:
```
$ catkin build camera_cone_detection
```

### Compilation configuration

`SGT_Macros.h`:
 * `CAMERA_DETECTION_CARSTATE` : publish camera pose (left eye) in `odom` frame to `/camera_pose` topic
 * `CAMERA_DETECTION_CAMERA_SHOW` : show live video stream with bounding boxes in a separate window (must be turned off in case of SSH access)
 * `CAMERA_DETECTION_FAKE_LIDAR` : publish position of detected cones on `/lidar_cones` topic
 * `CAMERA_DETECTION_CONSOLE_SHOW` : print detection results in terminal
 * `CAMERA_DETECTION_RECORD_VIDEO` : record output video stream (MP4)
 * `CAMERA_DETECTION_RECORD_VIDEO_SVO` : record output video (SVO) which can be used as input stream instead of live camera picture

Set the input stream in `CameraConeDetection.h` with variable `filename` value:
 * **"zed_camera"** : live camera picture
 * **"<path_to_.svo_file>"** : recorded SVO video

## Launch
```
  $ source ros_implementation/devel/setup.bash
  $ roslaunch camera_cone_detection camera_cone_detection.launch
```

### Launch configuration
In `param/camera_cone_detection.yaml` path to NN configuration files and output files can be set.

### RViz visualization
In new terminal run:
```
    $ source ros_implementation/devel/setup.bash
    $ roslaunch sensors_visualizator sensors_visualizator_camera.launch
```

 ## Visual odometry
 Node `visualOdometry` located in `visual_odometry` package subscribes `/camera_pose` topic from `cameraConeDetection` node and publishes transformation from `base_link` to `odom` frame on general `/tf` topic.


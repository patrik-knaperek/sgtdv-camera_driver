# camera_cone_detection

### Requirements

* Linux
* [**CUDA 10.0**](https://developer.nvidia.com/embedded/jetpack)
* [**OpenCV >= 4.0.1**](https://developer.nvidia.com/embedded/jetpack)
* [**ZED SDK 3.X**](https://www.stereolabs.com/developers/release/)
* [**Darknet**](https://github.com/AlexeyAB/darknet)  
  * Povolit v Makefile:
  * GPU=1
  * CUDNN=1
  * CUDNN_HALF=1
  * LIBSO=1


## Implementacia detekcie cez kameru v c++

v CMakeLists treba zmenit aktualnu cestu k lib darknetu

```
target_link_libraries(
    camera_cone_detection
    ${OpenCV_LIBS}
    ${ZED_LIBRARIES}
    /usr/src/ROS_implementation/darknet/libdarknet.so #treba zmenit na aktualnu cestu
    ${catkin_LIBRARIES}
)

```
treba zmenit cestu pre darknet aby sa vedelo skompilovat

skompilovat dany package sa da s ```catkin_make camera_cone_detection ``` v ros_implementation priecinku alebo ```catkin_make -j1 -l1```

nasledne spustit sa da v ```ros_implementation/devel/lib/camera_cone_detection```
```./camera_cone_detection``` musi mat pri sebe **yolov3-tiny.cfg**, **yolov3-tiny.weights**, **druha_jazda.svo** alebo to treba zmenit v ```CameraConeDetection.h``` ak ```filename``` zadame ako **"zed_camera"** tak by sa malo brat obraz z kamery

pre citanie ros sprav sa musi spustit
```. ros_implementation/devel/setup.bash ``` v ros_implementation priecinku a nasledne **rostopic echo nazov_spravy**

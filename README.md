# ROS_implementation

### Requirements

* Linux
* **CUDA 10.0**
* **OpenCV >= 4.0.1**
* **ZED SDK 3.X**:


#Implementacia detekcie cez karemu v c++

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
```./camera_cone_detection``` musi mat pri sebe **kuzel.names**, **yolov3-tiny.cfg**, **yolov3-tiny.weights**, **druha_jazda.svo** alebo to treba zmenit v ```CameraConeDetection.h``` ak ```filename``` zadame ako **"zed_camera"** tak by sa malo brat obraz z kamery

pre citanie ros sprav sa musi spustit
```. ros_implementation/devel/setup.bash ``` v ros_implementation priecinku a nasledne **rostopic echo nazov_spravy**

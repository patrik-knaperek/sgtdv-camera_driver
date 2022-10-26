# camera_cone_detection

### Requirements

* [**CUDA 10.0**](https://developer.nvidia.com/embedded/jetpack)
* [**OpenCV >= 4.0.1**](https://developer.nvidia.com/embedded/jetpack)
* [**ZED SDK 3.X**](https://www.stereolabs.com/developers/release/)
* [**Darknet**](https://github.com/AlexeyAB/darknet)  
  

### Kompilacia Darknetu
* Povolit v Makefile:
  * GPU=1
  * CUDNN=1
  * CUDNN_HALF=1
  * LIBSO=1
* Nasledne skompilovat pomocou `$ make`
* Vygenerovany subor `libdarknet.so` skopirovat do `camera_cone_detection/include`


## Implementacia detekcie cez kameru v C++

Konfiguracne subory pre neuronku, vygenerovane .weights a .svo subory su v adresari [**Darknet_cone_detection**](https://drive.google.com/drive/folders/144MJlPqqrMii9dVJtaWv_vCwrJNkGFed?usp=sharing) na G-Drive. Adresar treba skopirovat do `src/camera_cone_detection`. V pripade zmeny niektoreho suboru v nom, treba ho aktualizovat na G-Drive.

### Kompilacia
 * `$ cd /ros_implementation/src/`
 * `$ catkin build camera_cone_detection ` (ak nie je, treba predtym skompilovat aj `sgtdv_msgs`)

### Konfiguracia

V `SGT_Macros.h` je mozne vypnut/zapnut funkcionality (po kazdej zmene treba znovu skompilovat balicek)
 * `CAMERA_DETECTION_CARSTATE` : publikovanie udajov z odometrie na `visual_odometry` topic
 * `CAMERA_DETECTION_CAMERA_SHOW` : live zobrazenie videa z kamery spolu s bounding boxami v okne (pri ssh pristupe treba vypnut)
 * `CAMERA_DETECTION_FAKE_LIDAR` : publikovanie detekcii aj na `lidar_cones` topic
 * `CAMERA_DETECTION_CONSOLE_SHOW` : vypisovanie vysledkov detekcie do konzoly
 * `CAMERA_DETECTION_RECORD_VIDEO` : nahravanie vystupneho videa
 * `CAMERA_DETECTION_RECORD_VIDEO_SVO` : nahravanie videa vo formate, ktory sa da pouzit ako vstup miesto obrazu z kamery

V `param/camera_cone_detection.yaml` sa nastavuje cesta ku konfigurakom neuronky a vystupnym suborom (pri zapnutom nahravani)

Vstupny stream sa nastavuje pomocou premennej `filename` v subore `CameraConeDetection.h`:
 * **"zed_camera"**: live obraz z kamery
 * **"<path_to_.svo_file>"**: nahrate .svo video

### Spustenie

 * `$ . ros_implementation/devel/setup.bash`
 * `$ roslaunch camera_cone_detection camera_cone_detection.launch `


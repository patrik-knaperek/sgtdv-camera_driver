cmake_minimum_required(VERSION 3.0.2)
project(camera_driver)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
	roslib
  std_msgs
  sgtdv_msgs
  eigen_conversions
)

find_package(OpenCV REQUIRED)

message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

find_package(ZED 4 REQUIRED)
message(STATUS "ZED library status:")
message(STATUS "    version: ${ZED_VERSION}")
message(STATUS "    libraries: ${ZED_LIBRARIES}")
message(STATUS "    libraries path: ${ZED_LIBRARY_DIR}")
message(STATUS "    include path: ${ZED_INCLUDE_DIRS}")

find_package(CUDA REQUIRED)
message(STATUS "CUDA library status:")
message(STATUS "    version: ${CUDA_VERSION}")
message(STATUS "    libraries: ${CUDA_LIBRARIES}")
message(STATUS "    libraries path: ${CUDA_LIBRARY_DIR}")
message(STATUS "    include path: ${CUDA_INCLUDE_DIRS}")

find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS sgtdv_msgs
)

###########
## Build ##
###########

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  ${ZED_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}/../
)

add_executable(${PROJECT_NAME} src/${PROJECT_NAME}_node.cpp src/${PROJECT_NAME}.cpp)

target_link_libraries(
    ${PROJECT_NAME}
    ${OpenCV_LIBS}
    ${ZED_LIBRARIES}
    ${PROJECT_SOURCE_DIR}/include/libdarknet.so
    ${catkin_LIBRARIES}
)

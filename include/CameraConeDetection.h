#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <sgtdv_msgs/ConeArr.h>
#include <chrono>

constexpr float FPS = 30.f;
constexpr float TIME_PER_FRAME = 1.f / FPS;

class CameraConeDetection
{
public:
    CameraConeDetection();
    ~CameraConeDetection();

    void SetSignalPublisher(ros::Publisher signalPublisher);
    void SetConePublisher(ros::Publisher mainPublisher);
    void Do();

private:
    ros::Publisher m_signalPublisher;
    ros::Publisher m_conePublisher;
};
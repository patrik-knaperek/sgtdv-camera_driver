#include "../include/CameraConeDetection.h"

CameraConeDetection::CameraConeDetection()
{

}

CameraConeDetection::~CameraConeDetection()
{

}

void CameraConeDetection::SetSignalPublisher(ros::Publisher signalPublisher)
{
    m_signalPublisher = signalPublisher;
}

void CameraConeDetection::SetConePublisher(ros::Publisher conePublisher)
{
    m_conePublisher = conePublisher;
}

void CameraConeDetection::Do()
{
    while (ros::ok())
    {
        auto start = std::chrono::steady_clock::now();

        //m_signalPublisher.publish(new std_msgs::Empty);

        //TODO function body

        auto finish = std::chrono::steady_clock::now();
        float timeDiff = TIME_PER_FRAME - std::chrono::duration_cast<std::chrono::milliseconds>(finish - start).count() / 1000.f;

        if (timeDiff > 0.f)
        {
            sleep(timeDiff);
        }
        else
        {
            //defenzivne programovanie ftw
        }        
    }
}
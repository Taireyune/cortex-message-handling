#ifndef BLACKFLY_BINOCULAR_FEED
#define BLACKFLY_BINOCULAR_FEED

#include <Spinnaker.h>
#include <SpinGenApi/SpinnakerGenApi.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>

#include "cortex_message_handling/BinocularFeed.h"

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
using namespace std;

///// blackfly_trigger_aquisition.cpp
class Blackfly_trigger_aquisition
{
public:
    Blackfly_trigger_aquisition(CameraPtr camera_pointer);
    ~Blackfly_trigger_aquisition();

    int fire_trigger();
    cv::Mat grab_frame();  

private:
    // minimum milliseconds before next image
    const int grab_image_timeout_ = 150;

    // input image size is 1616 x 1240, crop to 1240 x 1240
    const cv::Rect cropROI_(188, 0, 1240, 1240);

    CameraPtr pCam_;
    INodeMap& nodeMap_;
    CCommandPtr ptrSoftwareTriggerCommand_;
    cv::Mat cv_image(cv::Size(512, 512), CV_8UC3, Scalar(0));

    int configure_camera_settings();
    int release_camera();
};

///// live_binocular_relay.cpp

class Live_binocular_relay
{
public:
    Live_binocular_relay();
    ~Live_binocular_relay();

    // methods
    // trigger and publish 
    int update_attributes();

private:
    SystemPtr system;
    CameraList camList;
    unsigned int numCameras;
    Blackfly_trigger_aquisition[2] camera_objects_;

    // ros objects
    ros::NodeHandle n;
    ros::Publisher video_feed_pub;
};

int main(int argc, char **argv);
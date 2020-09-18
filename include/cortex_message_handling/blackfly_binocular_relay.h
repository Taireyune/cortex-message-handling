#ifndef BLACKFLY_BINOCULAR_RELAY
#define BLACKFLY_BINOCULAR_RELAY

#include <Spinnaker.h>
#include <SpinGenApi/SpinnakerGenApi.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <chrono>

#include "cortex_message_handling/BinocularFeed.h"

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
using namespace std;

///// blackfly_trigger_acquisition.cpp
class Blackfly_trigger_acquisition
{
public:
    Blackfly_trigger_acquisition(CameraPtr camera_pointer);
    ~Blackfly_trigger_acquisition();

    int fire_trigger();
    cv::Mat grab_frame();  

    // cleanup
    int release_camera();

private:
    // minimum milliseconds before next image
    const int grab_image_timeout_ = 30;
    double exposure_time_ = 3000.0;

    // cv variables
    cv::Rect cropROI_;
    cv::Mat cv_image;

    // blackfly pointers
    CameraPtr pCam_;
    INodeMap* nodeMap_;
    CCommandPtr ptrSoftwareTriggerCommand_;

    // image format conversion
    void convert_to_cv_image(ImagePtr pImage);

    // setup
    int configure_camera_settings();
};

class Blackfly_binocular_relay
{
public:
    Blackfly_binocular_relay();
    ~Blackfly_binocular_relay();

    // methods
    // trigger and publish 
    int update_attributes();

    // cleanup
    int release_cameras();

private:
    SystemPtr system;
    CameraList camList;
    unsigned int numCameras;
    Blackfly_trigger_acquisition* camera_leftside_;

    // ros objects
    ros::NodeHandle n;
    ros::Publisher video_feed_pub;
};

#endif
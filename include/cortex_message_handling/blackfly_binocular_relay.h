#ifndef BLACKFLY_BINOCULAR_RELAY
#define BLACKFLY_BINOCULAR_RELAY

#include <Spinnaker.h>
#include <SpinGenApi/SpinnakerGenApi.h>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <thread>
#include <zmq.hpp>

#include "cortex_message_handling/BinocularFeed.h"

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
using namespace std;

///
///// blackfly_trigger_acquisition.cpp
///
class Blackfly_trigger_acquisition
{
/// *** properties ***
public:
    int cv_image_size();

protected: 
    /// camera setting values
    const int grab_image_timeout_ = 1000;
    double exposure_time_ = 5000.0;

    /// cv variables
    // input image size is 1616 x 1240, will be cropped to 1240 x 1240
    cv::Rect cropROI_{188, 0, 1240, 1240};
    cv::Mat cv_image_{cv::Size(1240, 1240), CV_8UC3, cv::Scalar(0)};

    /// resize to 512 x 512
    // cv_image_ = cv::Mat(cv::Size(512, 512), CV_8UC3, cv::Scalar(0));

    int cv_image_size_ = cv_image_.total() * cv_image_.elemSize();

    // blackfly pointers
    CameraPtr pCam_;
    INodeMap* nodeMap_;
    CCommandPtr ptrSoftwareTriggerCommand_;

/// *** methods ***
public:
    Blackfly_trigger_acquisition(CameraPtr camera_pointer);
    Blackfly_trigger_acquisition() = delete;
    ~Blackfly_trigger_acquisition();

    int fire_trigger();
    cv::Mat grab_frame();  

    // cleanup
    int release_camera();

protected:
    // image format conversion
    void convert_to_cv_image(ImagePtr pImage);

    // setup
    int configure_camera_settings();
};

class Blackfly_camera_finder
{
/// *** properties ***
private: 
    SystemPtr system;
    CameraList camList;
    unsigned int numCameras;

/// *** methods ***
public:   
    Blackfly_camera_finder();
    ~Blackfly_camera_finder();

    CameraPtr get_camera(int index);
};

class Blackfly_video_relay: public Blackfly_trigger_acquisition
{
    static void deallocator(void* data, void* hint);
/// *** properties ***
public:
    string relay_address();
    bool is_running();

protected:
    const int Loop_count_ = 1000;
    bool running_ = false;
    bool stop_loop_ = false;

    /// ros objects
    ros::NodeHandle node;

    /// zmq objects
    string relay_address_ = "ipc:///tmp/blackfly_video_relay";
    zmq::context_t context_;
    zmq::socket_t publisher_{context_, zmq::socket_type::pub};

/// *** methods ***
public:  
    Blackfly_video_relay() = delete;
    Blackfly_video_relay(CameraPtr camera_pointer, string address_suffix);
    ~Blackfly_video_relay();

    void start_acquisition(int rate);
    void stop_loop();
};

// class Live_binocular_relay
// {
// protected:
//     int rate;

// public:
//     Live_binocular_relay(Blackfly_video_relay camera_1)
// };

class Blackfly_binocular_relay
{
/// *** attributes ***
private:  
    bool start_trigger_loop_ = false;
    const int Loop_count_ = 1000;

    Blackfly_trigger_acquisition camera_leftside_;

    /// ros objects
    /// left camera
    ros::NodeHandle n_left_;
    // ros::Publisher video_feed_leftside_pub_;
    // image_transport::ImageTransport transporter_leftside_(n_left_);
    // image_transport::Publisher video_feed_leftside_pub_;

    /// right camera
    /// to be added

    /// zmq objects
    zmq::context_t context_;
    zmq::socket_t publisher_leftside_{context_, zmq::socket_type::pub};

/// *** methods ***
public:   
    Blackfly_binocular_relay() = delete;
    Blackfly_binocular_relay(CameraPtr camera_pointer);
    ~Blackfly_binocular_relay();

    // methods
    // trigger and publish 
    // int update_attributes();

    // run loops
    void run_multithread(int rate);
    void run_singlethread(int rate);

    // cleanup
    int release_cameras();

private:
    // multi threading
    void trigger_thread(int rate);
    void image_process_thread(int rate);
    void deallocator(void* data, void* hint);
};

#endif
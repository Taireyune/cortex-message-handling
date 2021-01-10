#include "cortex_message_handling/blackfly_binocular_relay.h"

///// ***Blackfly_trigger_acquisition***
/// Object for each blackfly camera to acquire image with trigger

///
///// Constructor
///
Blackfly_trigger_acquisition::Blackfly_trigger_acquisition(CameraPtr camera_pointer)
    : pCam_(camera_pointer)
{
    cout << "[Blackfly_trigger_acquisition::Blackfly_trigger_acquisition] Instantiating..." << endl;

    int result = 0;
   
    try
    {      
        pCam_->Init();
        INodeMap & nodeMap = pCam_->GetNodeMap();
        
    }

    catch (Spinnaker::Exception& e)
    {
        throw e; 
    }

    // configure camera
    result = configure_camera_settings();
    if (result == -1)
    {
        throw "[Blackfly_trigger_acquisition::configure_camera_settings] Unable to properly configure camera failed.";
    }

    cout << "[Blackfly_trigger_acquisition::Blackfly_trigger_acquisition] Instantiated." << endl;
}

///
///// Properties
///
int Blackfly_trigger_acquisition::cv_image_size()
{ 
    return cv_image_size_; 
}

///
///// Methods
///
///// configure camera settings
///
/// Set trigger to off
/// switch to TriggerOverlap, ReadOut
/// use software trigger
/// switch trigger backon
/// select continuous acquisition
/// start acquisition
///
int Blackfly_trigger_acquisition::configure_camera_settings()
{
    cout << endl << "*** CONFIGURING CAMERA ***" << endl << endl;
    try
    {
        INodeMap & nodeMap = pCam_->GetNodeMap();
        
        //
        // Turn off automatic exposure mode
        //
        // *** NOTES ***
        // Automatic exposure prevents the manual configuration of exposure
        // time and needs to be turned off.
        //
        // *** LATER ***
        // Exposure time can be set automatically or manually as needed. This
        // example turns automatic exposure off to set it manually and back
        // on in order to return the camera to its default state.
        //
        CEnumerationPtr ptrExposureAuto = nodeMap.GetNode("ExposureAuto");
        if (!IsAvailable(ptrExposureAuto) || !IsWritable(ptrExposureAuto))
        {
            cout << "Unable to disable automatic exposure (node retrieval). Aborting..." << endl << endl;
            return -1;
        }
        CEnumEntryPtr ptrExposureAutoOff = ptrExposureAuto->GetEntryByName("Off");
        if (!IsAvailable(ptrExposureAutoOff) || !IsReadable(ptrExposureAutoOff))
        {
            cout << "Unable to disable automatic exposure (enum entry retrieval). Aborting..." << endl << endl;
            return -1;
        }
        ptrExposureAuto->SetIntValue(ptrExposureAutoOff->GetValue());
        cout << "Automatic exposure disabled..." << endl;
        //
        // Set exposure time manually; exposure time recorded in microseconds
        //
        // *** NOTES ***
        // The node is checked for availability and writability prior to the
        // setting of the node. Further, it is ensured that the desired exposure
        // time does not exceed the maximum. Exposure time is counted in
        // microseconds. This information can be found out either by
        // retrieving the unit with the GetUnit() method or by checking SpinView.
        //
        CFloatPtr ptrExposureTime = nodeMap.GetNode("ExposureTime");
        if (!IsAvailable(ptrExposureTime) || !IsWritable(ptrExposureTime))
        {
            cout << "Unable to set exposure time. Aborting..." << endl << endl;
            return -1;
        }
        // Ensure desired exposure time does not exceed the maximum
        const double exposureTimeMax = ptrExposureTime->GetMax();
        if (exposure_time_ > exposureTimeMax)
        {
            exposure_time_ = exposureTimeMax;
        }
        ptrExposureTime->SetValue(exposure_time_);
        cout << std::fixed << "Exposure time set to " << exposure_time_ << " us..." << endl;

        //
        ///// Ensure trigger mode off
        //
        // *** NOTES ***
        // The trigger must be disabled in order to configure whether the source
        // is software or hardware.
        //
        CEnumerationPtr ptrTriggerMode = nodeMap.GetNode("TriggerMode");
        if (!IsAvailable(ptrTriggerMode) || !IsReadable(ptrTriggerMode))
        {
            cout << "Unable to disable trigger mode (node retrieval). Aborting..." << endl;
            return -1;
        }

        CEnumEntryPtr ptrTriggerModeOff = ptrTriggerMode->GetEntryByName("Off");
        if (!IsAvailable(ptrTriggerModeOff) || !IsReadable(ptrTriggerModeOff))
        {
            cout << "Unable to disable trigger mode (enum entry retrieval). Aborting..." << endl;
            return -1;
        }

        ptrTriggerMode->SetIntValue(ptrTriggerModeOff->GetValue());
        cout << "Trigger mode disabled..." << endl;

        //
        ///// Select trigger source
        //
        // *** NOTES ***
        // The trigger source must be set to hardware or software while trigger
        // mode is off.
        //
        CEnumerationPtr ptrTriggerSource = nodeMap.GetNode("TriggerSource");
        if (!IsAvailable(ptrTriggerSource) || !IsWritable(ptrTriggerSource))
        {
            cout << "Unable to set trigger mode (node retrieval). Aborting..." << endl;
            return -1;
        }

        // Set trigger mode to software
        CEnumEntryPtr ptrTriggerSourceSoftware = ptrTriggerSource->GetEntryByName("Software");
        if (!IsAvailable(ptrTriggerSourceSoftware) || !IsReadable(ptrTriggerSourceSoftware))
        {
            cout << "Unable to set trigger mode (enum entry retrieval). Aborting..." << endl;
            return -1;
        }

        ptrTriggerSource->SetIntValue(ptrTriggerSourceSoftware->GetValue());
        cout << "Trigger source set to software." << endl;

        //
        /// Set TriggerSelector to TriggerOverlap
        //
        // *** NOTES ***
        // For this example, the trigger selector should be set to frame start.
        // This is the default for most cameras.
        //
        CEnumerationPtr ptrTriggerOverlap = nodeMap.GetNode("TriggerOverlap");
        if (!IsAvailable(ptrTriggerOverlap) || !IsWritable(ptrTriggerOverlap))
        {
            cout << "Unable to set trigger overlap (node retrieval). Aborting..." << endl;
            return -1;
        }

        CEnumEntryPtr ptrTriggerOverlapReadOut = ptrTriggerOverlap->GetEntryByName("ReadOut");
        if (!IsAvailable(ptrTriggerOverlapReadOut) || !IsReadable(ptrTriggerOverlapReadOut))
        {
            cout << "Unable to set trigger overlap (enum entry retrieval). Aborting..." << endl;
            return -1;
        }

        ptrTriggerOverlap->SetIntValue(ptrTriggerOverlapReadOut->GetValue());
        cout << "Trigger overlap set to ReadOut." << endl;

        //
        // Turn trigger mode on
        //
        // *** LATER ***
        // Once the appropriate trigger source has been set, turn trigger mode
        // on in order to retrieve images using the trigger.
        //
        CEnumEntryPtr ptrTriggerModeOn = ptrTriggerMode->GetEntryByName("On");
        if (!IsAvailable(ptrTriggerModeOn) || !IsReadable(ptrTriggerModeOn))
        {
            cout << "Unable to enable trigger mode (enum entry retrieval). Aborting..." << endl;
            return -1;
        }

        ptrTriggerMode->SetIntValue(ptrTriggerModeOn->GetValue());
        cout << "Trigger mode turned back on." << endl << endl;

        //
        ///// Set acquisition mode to continuous
        //
        CEnumerationPtr ptrAcquisitionMode = nodeMap.GetNode("AcquisitionMode");
        if (!IsAvailable(ptrAcquisitionMode) || !IsWritable(ptrAcquisitionMode))
        {
            cout << "Unable to set acquisition mode to continuous (node retrieval). Aborting..." << endl << endl;
            return -1;
        }

        CEnumEntryPtr ptrAcquisitionModeContinuous = ptrAcquisitionMode->GetEntryByName("Continuous");
        if (!IsAvailable(ptrAcquisitionModeContinuous) || !IsReadable(ptrAcquisitionModeContinuous))
        {
            cout << "Unable to set acquisition mode to continuous (entry 'continuous' retrieval). Aborting..." << endl
                 << endl;
            return -1;
        }

        int64_t acquisitionModeContinuous = ptrAcquisitionModeContinuous->GetValue();
        ptrAcquisitionMode->SetIntValue(acquisitionModeContinuous);
        cout << "Acquisition mode set to continuous." << endl;

        // setup trigger pointer
        ptrSoftwareTriggerCommand_ = nodeMap.GetNode("TriggerSoftware");
        if (!Spinnaker::GenApi::IsAvailable(ptrSoftwareTriggerCommand_) || !IsWritable(ptrSoftwareTriggerCommand_))
        {
            cout << "Unable to obtain trigger pointer. Aborting..." << endl;
            return -1;
        }
        
        cout << "Software trigger pointer obtained." << endl;

        // Begin acquiring images
        pCam_->BeginAcquisition();
        ptrSoftwareTriggerCommand_->Execute(); // trigger once so there is something in the buffer
        cout << "Begin acquiring images..." << endl;
    }
    catch (Spinnaker::Exception& e)
    {
        cout << "[Blackfly_trigger_acquisition::configure_camera_settings] Error: " << e.what() << endl;
        return -1;
    }

    return 0;
}

///
///// Send fire trigger signal to camera
///
int Blackfly_trigger_acquisition::fire_trigger()
{
    // make sure the trigger pointer doesnt need to be renewed every frame
    ptrSoftwareTriggerCommand_->Execute();
    // cout << "[Blackfly_trigger_acquisition::fire_trigger] fired." << endl;
}

///
///// Get image from buffer
///
/// Get image pointer from buffer
/// convert to opencv format image
/// image is copy to cv_image_
///
cv::Mat Blackfly_trigger_acquisition::grab_frame()
{
    // Retrieve next image
    ImagePtr pResultImage = pCam_ -> GetNextImage(grab_image_timeout_);

    if (pResultImage->IsIncomplete())
    {
        cout << "Image incomplete with image status " << pResultImage->GetImageStatus() << "..." << endl
                << endl;
    }
    else
    {
        convert_to_cv_image(pResultImage);
    }
    // cout << "[Blackfly_trigger_acquisition::grab_frame] image updated." << endl;

    // Release image
    pResultImage->Release();
    return cv_image_;
}

///
////// Convert image pointer to opencv matrix
///
/// convert
/// do some cropping
/// additional modifcations can be done
/// image copied to cv_image_
///
void Blackfly_trigger_acquisition::convert_to_cv_image(ImagePtr pImage) 
{

    ImagePtr convertedImage = pImage->Convert(PixelFormat_BGR8); //, NEAREST_NEIGHBOR);
    unsigned int XPadding = convertedImage->GetXPadding();
    unsigned int YPadding = convertedImage->GetYPadding();
    unsigned int rowsize = convertedImage->GetWidth();
    unsigned int colsize = convertedImage->GetHeight();

    /// here is where code can be added to modify image
    /// remember to change cv_image if modifications are made
    /// image data contains padding. When allocating Mat container size, you need to account for the X,Y image data padding.
    
    // cv::Mat mat;
    // mat = cv::Mat(colsize + YPadding, rowsize + XPadding, CV_8UC3, convertedImage->GetData(), convertedImage->GetStride());
    cv::Mat mat(colsize + YPadding, rowsize + XPadding, CV_8UC3, convertedImage->GetData(), convertedImage->GetStride());

    /// Resize based on size of cv_image_.
    /// Data is copied to cv_image_.
    cv::resize(mat(cropROI_), cv_image_, cv_image_.size());
}

///
///// Destructor
///
Blackfly_trigger_acquisition::~Blackfly_trigger_acquisition()
{
    release_camera();
    // cout << "[Blackfly_trigger_acquisition::~Blackfly_trigger_acquisition] Destructor called." << endl;
}

///
///// Camera cleanup
///
/// turn trigger mode backoff
/// set to automatic exposure
/// de-initialize, release pointer
///
int Blackfly_trigger_acquisition::release_camera()
{   
    try
    {
        INodeMap & nodeMap = pCam_->GetNodeMap();
        pCam_->EndAcquisition();   

        //
        // Turn trigger mode back off
        //
        // *** NOTES ***
        // Once all images have been captured, turn trigger mode back off to
        // restore the camera to a clean state.
        //
        CEnumerationPtr ptrTriggerMode = nodeMap.GetNode("TriggerMode");
        if (!IsAvailable(ptrTriggerMode) || !IsReadable(ptrTriggerMode))
        {
            cout << "Unable to disable trigger mode (node retrieval). Non-fatal error..." << endl;
            return -1;
        }

        CEnumEntryPtr ptrTriggerModeOff = ptrTriggerMode->GetEntryByName("Off");
        if (!IsAvailable(ptrTriggerModeOff) || !IsReadable(ptrTriggerModeOff))
        {
            cout << "Unable to disable trigger mode (enum entry retrieval). Non-fatal error..." << endl;
            return -1;
        }

        ptrTriggerMode->SetIntValue(ptrTriggerModeOff->GetValue());

        cout << "Trigger mode disabled..." << endl;

        //
        // Turn automatic exposure back on
        //
        // *** NOTES ***
        // Automatic exposure is turned on in order to return the camera to its
        // default state.
        //
        CEnumerationPtr ptrExposureAuto = nodeMap.GetNode("ExposureAuto");
        if (!IsAvailable(ptrExposureAuto) || !IsWritable(ptrExposureAuto))
        {
            cout << "Unable to enable automatic exposure (node retrieval). Non-fatal error..." << endl << endl;
            return -1;
        }
        CEnumEntryPtr ptrExposureAutoContinuous = ptrExposureAuto->GetEntryByName("Continuous");
        if (!IsAvailable(ptrExposureAutoContinuous) || !IsReadable(ptrExposureAutoContinuous))
        {
            cout << "Unable to enable automatic exposure (enum entry retrieval). Non-fatal error..." << endl << endl;
            return -1;
        }
        ptrExposureAuto->SetIntValue(ptrExposureAutoContinuous->GetValue());
        cout << "Automatic exposure enabled..." << endl;
    }
    catch (Spinnaker::Exception& e)
    {
        // cout << "Error: " << e.what() << endl;
        return -1;
    }

    try
    {
        pCam_->DeInit();
    }
    catch (Spinnaker::Exception& e)
    {
        // cout << "Error: " << e.what() << endl;
        return -1;
    }
    
    cout << "[Blackfly_trigger_acquisition::release_camera] Cameras released." << endl;
    return 0;
}

///// *** Blackfly_camera_finder ***
/// Find the blackfly cameras from USB port
///
///// Constructor
///
Blackfly_camera_finder::Blackfly_camera_finder()
{
    ///// find cameras
    // Retrieve singleton reference to system object
    system = System::GetInstance();
    camList = system->GetCameras();
    numCameras = camList.GetSize();

    // Finish if there are no cameras
    if (numCameras == 0)
    {
        // Clear camera list before releasing system
        camList.Clear();
        // Release system
        system->ReleaseInstance();

        cout << "Not enough cameras!" << endl;
        throw "[Blackfly_camera_finder::Blackfly_camera_finder] Abort! Camera not found.";
    }
}

///
///// Destructor
///
/// release cameras
///
Blackfly_camera_finder::~Blackfly_camera_finder()
{
    camList.Clear();
    system->ReleaseInstance();
    cout << "[Blackfly_camera_finder::~Blackfly_camera_finder] Camera list cleared. System released." << endl;
}

///
///// Get the camera by index
///
CameraPtr Blackfly_camera_finder::get_camera(int index)
{
    return camList.GetByIndex(index);
}

///// *** Blackfly_video_relay ***
/// ROS node for relaying images from camera to other nodes
/// Use ZeroMQ library to improve image sending through IPC
/// Use ROS library to handle loop
///
///// Constructor
///
Blackfly_video_relay::Blackfly_video_relay(CameraPtr camera_pointer, string address_suffix)
    : Blackfly_trigger_acquisition(camera_pointer)
{
    /// zmq publisher
    relay_address_ = relay_address_ + address_suffix;
    publisher_.setsockopt(ZMQ_CONFLATE, 1);
    publisher_.bind(relay_address_);
    cout << "[Blackfly_video_relay::Blackfly_video_relay] Publisher is binded to " << relay_address_ << endl;
}

///
///// Properties
///
string Blackfly_video_relay::relay_address()
{
    return relay_address_;
}

bool Blackfly_video_relay::is_running()
{
    return running_;
}

///
///// Methods
///
///// Main loop for image acquisition
///
/// while running the loop
/// obtain image from camera
/// send image through zmq publisher
void Blackfly_video_relay::start_acquisition(int rate)
{
    ros::Rate loop_rate(rate);
    ros::Time time_now = ros::Time::now();
    running_ = true;
    string frame_rate;
    while (ros::ok() && !stop_loop_)
    {
        /// timer
        frame_rate = "Frame rate : " + to_string(1 / (ros::Time::now() - time_now).toSec()) + " hz";
        cout << frame_rate << endl;
        time_now = ros::Time::now();

        // when there are multiple cameras, remove trigger from loop 
        Blackfly_trigger_acquisition::fire_trigger();
        Blackfly_trigger_acquisition::grab_frame(); 
        cv::putText(
            Blackfly_trigger_acquisition::cv_image_,
            frame_rate,
            cv::Point(10, 30),
            cv::FONT_HERSHEY_DUPLEX,
            1,
            CV_RGB(255, 255, 255),
            2
        );

        /// for opencv
        cv::imshow("Display window", Blackfly_trigger_acquisition::cv_image_);
        if (cv::waitKey(10) == 27)
        {
            cout << "[Blackfly_binocular_relay::run_singlethread] exit command." << endl;
            break;
        }  

        zmq::message_t msg(Blackfly_trigger_acquisition::cv_image_.data, Blackfly_trigger_acquisition::cv_image_size());
        // zmq::message_t msg;
        // zmq::message_t init_msg(
        //     (void*) Blackfly_trigger_acquisition::cv_image_.data, 
        //     Blackfly_trigger_acquisition::cv_image_size(),
        //     Blackfly_video_relay::deallocator
        // );

        bool ret;
        ret = publisher_.send(msg, ZMQ_DONTWAIT);
        // try
        // {
        //     ret = publisher_leftside_.send(msg, 1);
        // }
        // catch(const exception& e)
        // {
        //     cout << "E: connect failed with error " << e.what() << endl;
        // }
        
        if (!ret)
        {
            cout << "[Blackfly_video_relay::start_acquisition] publisher send failed." << endl;
        }
        // cout << "[Blackfly_video_relay::start_acquisition] after sent." << endl;

        loop_rate.sleep();                 
    }

    running_ = false;
}

///
///// deallocation of image buffer after publisher sends the image
///
void Blackfly_video_relay::deallocator(void* data, void* hint)
{
    // free(data);
}

///
///// Stop the image acquisition loop
///
void Blackfly_video_relay::stop_loop()
{
    while (!stop_loop_)
    {
        stop_loop_ = true;
    }
}

///
///// Destructor
///
Blackfly_video_relay::~Blackfly_video_relay()
{
    stop_loop();
    Blackfly_trigger_acquisition::release_camera();
}

///// *** Blackfly_binocular_relay ***
/// ROS node for relaying images from camera to other nodes
/// Currently only handles one camera
/// Modify this for multiple cameras
///
///// Constructor
///
Blackfly_binocular_relay::Blackfly_binocular_relay(CameraPtr camera_pointer)
    : camera_leftside_(camera_pointer)
{
    // initialize node
    //
    // left side
    string binocular_feed_leftside_identifier = "px150/binocular_feed_leftside_R";
    // video_feed_leftside_pub_ = n_left_.advertise<sensor_msgs::Image>(binocular_feed_leftside_identifier, 1);
    // video_feed_leftside_pub_ = transporter_leftside_.advertise(binocular_feed_identifier, 1);
    
    
    // right side
    // to be added

    /// zmq piping
    string address_leftside = "ipc:///tmp/binocular_feed_leftside_R";
    publisher_leftside_.bind(address_leftside);

    cout << "[Blackfly_binocular_relay::Blackfly_binocular_relay] Instantiated." << endl;
}

///
///// Destructory
///
Blackfly_binocular_relay::~Blackfly_binocular_relay()
{
    cout << "[Blackfly_binocular_relay::~Blackfly_binocular_relay] Relay object terminated." << endl;
}


// int Blackfly_binocular_relay::update_attributes()
// {
//     camera_leftside_.fire_trigger(); 
//     cv::Mat frame = camera_leftside_.grab_frame();
//     cv::imshow( "Display window", frame);  
//     ros::spinOnce();
//     return 0;
// }

///
///// Release camera when finished
///
/// remember to add the other cameras when there is more than one
///
int Blackfly_binocular_relay::release_cameras()
{
    try
    {
        publisher_leftside_.close();
        context_.close();
    }
    catch(const exception& e)
    {
        cout << "E: cleanup failed " << e.what() << endl;
    }

    camera_leftside_.release_camera();
    return 0; 
}

void Blackfly_binocular_relay::deallocator(void* data, void* hint)
{
    free(data);
}
///
///// single thread node loop
///
void Blackfly_binocular_relay::run_singlethread(int rate)
{
    ros::Rate loop_rate(rate);
    chrono::steady_clock::time_point time_now = chrono::steady_clock::now();

    int count = 0;
    
    
    // while (count < Loop_count_)
    while (true)
    {
        // timer
        cout << "loop rate = " 
            << 1000000.0 / chrono::duration_cast<chrono::microseconds>(chrono::steady_clock::now() - time_now).count() 
            << " hz" << endl;
        time_now = chrono::steady_clock::now();

        camera_leftside_.fire_trigger();
        cv::Mat frame = camera_leftside_.grab_frame();

        /// for opencv
        cv::imshow("Display window", frame);
        if (cv::waitKey(10) == 27)
        {
            cout << "[Blackfly_binocular_relay::run_singlethread] exit command." << endl;
            break;
        }  

        
        // zmq::message_t msg;
        // void* hint = NULL;
        // zmq::message_t mess(
        //     (void*) frame.data, 
        //     camera_leftside_.cv_image_size(), 
        //     deallocator,
        //     hint
        // )
        zmq::message_t msg(frame.data, camera_leftside_.cv_image_size());
        // memcpy(msg.data(), frame.data, camera_leftside_.cv_image_size());
        
        bool ret;
        ret = publisher_leftside_.send(msg, 1);
        // try
        // {
        //     ret = publisher_leftside_.send(msg, 1);
        // }
        // catch(const exception& e)
        // {
        //     cout << "E: connect failed with error " << e.what() << endl;
        // }
        
        if (!ret)
        {
            cout << "[Blackfly_binocular_relay::run_singlethread] publisher send failed." << endl;
        }
        cout << "[Blackfly_binocular_relay::run_singlethread] after sent." << endl;

        count++;
        loop_rate.sleep();   
    }  
}

///
///// multi thread node loop
///
void Blackfly_binocular_relay::run_multithread(int rate)
{
    thread trigger_loop(&Blackfly_binocular_relay::trigger_thread, this, rate);
    thread image_loop(&Blackfly_binocular_relay::image_process_thread, this, rate);

    trigger_loop.join();
    image_loop.join();
}

///
///// trigger loop for multithread
///
void Blackfly_binocular_relay::trigger_thread(int rate)
{
    ros::Rate loop_rate(rate);
    chrono::steady_clock::time_point time_now = chrono::steady_clock::now();

    int count = 0;

    // wait for image_process_thread to finish initializing
    while (!start_trigger_loop_)
    {
        this_thread::sleep_for(std::chrono::microseconds(100));
    }
    while (count < Loop_count_)
    {
        // timer
        cout << "Trigger rate = " 
            << 1000000.0 / chrono::duration_cast<chrono::microseconds>(chrono::steady_clock::now() - time_now).count() 
            << " hz" << endl;
        time_now = chrono::steady_clock::now();

        camera_leftside_.fire_trigger();
        count++;
    
        // for opencv
        if (cv::waitKey(10) == 27)
        {
            cout << "[Blackfly_binocular_relay::trigger_thread] exit command." << endl;
            break;
        }        

        loop_rate.sleep();   
    }  

    cout << "[Blackfly_binocular_relay::trigger_thread] Thread finished." << endl;  
}

///
///// image process loop for multithread
///
void Blackfly_binocular_relay::image_process_thread(int rate)
{
    ros::Rate loop_rate(rate);
    chrono::steady_clock::time_point time_now = chrono::steady_clock::now();

    int count = 0;
    start_trigger_loop_ = true;
    while (count < Loop_count_)
    {
        // timer
        // cout << "Framerate = " 
        //     << 1000000.0 / chrono::duration_cast<chrono::microseconds>(chrono::steady_clock::now() - time_now).count() 
        //     << " hz" << endl;
        // time_now = chrono::steady_clock::now();

        cv::Mat frame = camera_leftside_.grab_frame();
        cv::imshow("Display window", frame);

        count++;

        // for opencv
        if (cv::waitKey(10) == 27)
        {
            cout << "[Blackfly_binocular_relay::image_process_thread] exit command." << endl;
            break;
        }

        loop_rate.sleep();          
    }

    cout << "[Blackfly_binocular_relay::image_process_thread] Thread finished." << endl;  
}
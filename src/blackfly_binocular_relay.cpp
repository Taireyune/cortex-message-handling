#include "cortex_message_handling/blackfly_binocular_relay.h"

Blackfly_trigger_acquisition::Blackfly_trigger_acquisition(CameraPtr camera_pointer)
{
    int result = 0;
    pCam_ = camera_pointer;

    // input image size is 1616 x 1240, will be cropped to 1240 x 1240
    cropROI_ = cv::Rect(188, 0, 1240, 1240);

    // resize to 512 x 512
    cv_image = cv::Mat(cv::Size(512, 512), CV_8UC3, cv::Scalar(0));

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

    cout << "[Blackfly_trigger_acquisition::Blackfly_trigger_acquisition] Object instantiated." << endl;
}

///// configure camera settings
// Set trigger to off
// switch to TriggerOverlap
// use software trigger
// switch trigger backon
//
// select continuous acquisition
// start acquisition
int Blackfly_trigger_acquisition::configure_camera_settings()
{
    cout << endl << endl << "*** CONFIGURING CAMERA ***" << endl << endl;
    try
    {
        INodeMap & nodeMap = pCam_->GetNodeMap();
        
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
        cout << "Trigger source set to software..." << endl;

        //
        ///// Set TriggerSelector to TriggerOverlap
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
        cout << "Trigger overlap set to ReadOut..." << endl;

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
        cout << "Trigger mode turned back on..." << endl << endl;

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
        cout << "Acquisition mode set to continuous..." << endl;

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
        cout << "Begin acquiring images..." << endl;
    }
    catch (Spinnaker::Exception& e)
    {
        cout << "[Blackfly_trigger_acquisition::configure_camera_settings] Error: " << e.what() << endl;
        return -1;
    }

    return 0;
}

int Blackfly_trigger_acquisition::fire_trigger()
{
    // make sure the trigger pointer doesnt need to be renewed every frame
    ptrSoftwareTriggerCommand_->Execute();
    // cout << "[Blackfly_trigger_acquisition::fire_trigger] fired.";
}

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
    
    // Release image
    pResultImage->Release();
    
    // cout << "[Blackfly_trigger_acquisition::grab_frame] Image obtained." << endl;
    return cv_image;
}


void Blackfly_trigger_acquisition::convert_to_cv_image(ImagePtr pImage) 
{
    ImagePtr convertedImage;
    convertedImage = pImage->Convert(PixelFormat_BGR8); //, NEAREST_NEIGHBOR);
    unsigned int XPadding = convertedImage->GetXPadding();
    unsigned int YPadding = convertedImage->GetYPadding();
    unsigned int rowsize = convertedImage->GetWidth();
    unsigned int colsize = convertedImage->GetHeight();
    
    //image data contains padding. When allocating Mat container size, you need to account for the X,Y image data padding.
    cv::Mat img;
    img = cv::Mat(colsize + YPadding, rowsize + XPadding, CV_8UC3, convertedImage->GetData(), convertedImage->GetStride());
    
    // crop, cropROI defined in header
    img = img(cropROI_);

    // resize image
    cv::resize(img, cv_image, cv_image.size());
}


Blackfly_trigger_acquisition::~Blackfly_trigger_acquisition()
{
    cout << "[Blackfly_trigger_acquisition::~Blackfly_trigger_acquisition] Destructor called." << endl;
}

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

        cout << "Trigger mode disabled..." << endl << endl;
    }
    catch (Spinnaker::Exception& e)
    {
        cout << "Error: " << e.what() << endl;
        return -1;
    }

    pCam_->DeInit();
    return 0;
}

Blackfly_binocular_relay::Blackfly_binocular_relay()
{
    // initialize node
    string binocular_feed_identifier = "px150/binocular_feed_R";
    video_feed_pub = n.advertise<cortex_message_handling::BinocularFeed>(binocular_feed_identifier, 1);
    
    ///// initialize camera
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
        throw "[Blackfly_binocular_relay::Blackfly_binocular_relay] Camera not found.";
    }

    // initialize blackfly cam objects
    for (unsigned int i = 0; i < numCameras; i++)
    {
        camera_objects_.push_back(Blackfly_trigger_acquisition(camList.GetByIndex(i)));
    }
}

Blackfly_binocular_relay::~Blackfly_binocular_relay()
{
}

int Blackfly_binocular_relay::update_attributes()
{
    // fire trigger
    camera_objects_[0].fire_trigger(); 
    
    cv::imshow( "Display window", camera_objects_[0].grab_frame());  
    ros::spinOnce();
    return 0;
}

int Blackfly_binocular_relay::release_cameras()
{
    try
    {
        // camera cleanup
        for (unsigned int i = 0; i < numCameras; i++)
        {
            camera_objects_[i].release_camera();
        }
        camera_objects_.clear();

        camList.Clear();
        system->ReleaseInstance();
        cout << "[Blackfly_binocular_relay::~Blackfly_binocular_relay] Cameras released." << endl;
        return 0;
    }
    catch(const exception& e)
    {
        cerr << e.what() << '\n';
        return -1;
    }  
}

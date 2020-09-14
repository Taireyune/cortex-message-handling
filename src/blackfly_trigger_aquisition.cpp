#include "cortex_message_handling/blackfly_binocular_feed.h"

Blackfly_trigger_aquisition::Blackfly_trigger_aquisition(CameraPtr camera_pointer)
{
    int result = 0;
    pCam_ = camera_pointer;

    try
    {      
        pCam_->Init();
        nodeMap_ = pCam_->GetNodeMap();
        
        // configure camera
        result = configure_camera_settings(nodeMap_);

        // setup trigger pointer
        ptrSoftwareTriggerCommand_ = nodeMap_.GetNode("TriggerSoftware");
        if (!Spinnaker::GenApi::IsAvailable(ptrSoftwareTriggerCommand_) || !IsWritable(ptrSoftwareTriggerCommand_))
        {
            cout << "Unable to execute trigger. Aborting..." << endl;
            result = -1;
        }
    }

    catch (Spinnaker::Exception& e)
    {
        throw e; 
    }
    
    if result == -1:
    {
        throw "[Blackfly_trigger_aquisition::Blackfly_trigger_aquisition] Constructor failed.";
    }
}

///// configure camera settings
// Set trigger to off
// switch to TriggerOverlap
// use software trigger
// switch trigger backon
//
// select continuous acquisition
// start acquisition
int Blackfly_trigger_aquisition::configure_camera_settings()
{
    int result = 0;

    cout << endl << endl << "*** CONFIGURING TRIGGER ***" << endl << endl;
    cout << "Software trigger chosen..." << endl;

    try
    {
        //
        ///// Ensure trigger mode off
        //
        // *** NOTES ***
        // The trigger must be disabled in order to configure whether the source
        // is software or hardware.
        //
        CEnumerationPtr ptrTriggerMode = nodeMap_.GetNode("TriggerMode");
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
        ///// Set TriggerSelector to TriggerOverlap
        //
        // *** NOTES ***
        // For this example, the trigger selector should be set to frame start.
        // This is the default for most cameras.
        //
        CEnumerationPtr ptrTriggerSelector = nodeMap_.GetNode("TriggerSelector");
        if (!IsAvailable(ptrTriggerSelector) || !IsWritable(ptrTriggerSelector))
        {
            cout << "Unable to set trigger selector (node retrieval). Aborting..." << endl;
            return -1;
        }

        CEnumEntryPtr ptrTriggerSelectorTriggerOverlap = ptrTriggerSelector->GetEntryByName("triggerOverlap");
        if (!IsAvailable(ptrTriggerSelectorTriggerOverlap) || !IsReadable(ptrTriggerSelectorTriggerOverlap))
        {
            cout << "Unable to set trigger selector (enum entry retrieval). Aborting..." << endl;
            return -1;
        }

        ptrTriggerSelector->SetIntValue(ptrTriggerSelectorTriggerOverlap->GetValue());
        cout << "Trigger selector set to frame start..." << endl;

        //
        ///// Select trigger source
        //
        // *** NOTES ***
        // The trigger source must be set to hardware or software while trigger
        // mode is off.
        //
        CEnumerationPtr ptrTriggerSource = nodeMap_.GetNode("TriggerSource");
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
        CEnumerationPtr ptrAcquisitionMode = nodeMap_.GetNode("AcquisitionMode");
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

        // Begin acquiring images
        pCam_->BeginAcquisition();
        cout << "Acquiring images..." << endl;

    }
    catch (Spinnaker::Exception& e)
    {
        cout << "Error: " << e.what() << endl;
        result = -1;
    }

    return result;
}

int Blackfly_trigger_aquisition::fire_trigger()
{
    // make sure the trigger pointer doesnt need to be renewed every frame
    ptrSoftwareTriggerCommand_->Execute();
}

cv::Mat Blackfly_trigger_aquisition::grab_frame()
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
        // edit
        convert_to_cv_image(pResultImage);
    }
    
    // Release image
    pResultImage->Release();
    
    return cv_image;
}


void Blackfly_trigger_aquisition::convert_to_cv_image(ImagePtr pImage) 
{
    ImagePtr convertedImage;
    convertedImage = pImage->Convert(PixelFormat_BGR8); //, NEAREST_NEIGHBOR);
    unsigned int XPadding = convertedImage->GetXPadding();
    unsigned int YPadding = convertedImage->GetYPadding();
    unsigned int rowsize = convertedImage->GetWidth();
    unsigned int colsize = convertedImage->GetHeight();
    //image data contains padding. When allocating Mat container size, you need to account for the X,Y image data padding.
    Mat img;
    img = Mat(colsize + YPadding, rowsize + XPadding, CV_8UC3, convertedImage->GetData(), convertedImage->GetStride());
    
    // crop, cropROI defined in header
    img = img(cropROI_);

    // resize image
    cv::resize(img, cv_image, cv_image.size());
}


Blackfly_trigger_aquisition::~Blackfly_trigger_aquisition()
{
    release_camera();
}

int Blackfly_trigger_aquisition::release_camera()
{   

    int result = 0;

    try
    {
        //
        // Turn trigger mode back off
        //
        // *** NOTES ***
        // Once all images have been captured, turn trigger mode back off to
        // restore the camera to a clean state.
        //
        CEnumerationPtr ptrTriggerMode = nodeMap_.GetNode("TriggerMode");
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
        result = -1;
    }

    pCam_ -> DeInit();
    return result;
}


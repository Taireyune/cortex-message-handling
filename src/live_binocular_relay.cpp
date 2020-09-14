#include "blackfly_binocular_feed.h"

Live_binocular_relay::Live_binocular_relay()
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
        throw "[Live_binocular_relay::Live_binocular_relay] Camera not found.";
    }

    // initialize blackfly cam objects
    for (unsigned int i = 0; i < numCameras; i++)
    {
        camera_objects_[i] = new Blackfly_trigger_aquisition(camList.GetByIndex(i));
    }
}

int Live_binocular_relay::update_attributes()
{
    // fire trigger
    camera_objects_[0].fire_trigger(); 
    
    cv::imshow( "Display window", camera_objects_[0].grab_frame());  
    ros::spinOnce();
    return 0;
}

Live_binocular_relay::~Live_binocular_relay()
{
    // camera cleanup
    delete[] camera_objects_;

    camList.Clear();
    system->ReleaseInstance();
    cout << "[Live_binocular_relay::~Live_binocular_relay] Cameras released." << endl;
}

int main(int argc, char **argv)
{
    // ros init
    ros::init(argc, argv, "Live_binocular_relay");
    
    // initialize nodes and camera
    Live_binocular_relay relay;
    
    ros::Rate loop_rate(10);

    int count = 0;
    while (count < 100)
    {
        relay.update_attributes();
        count += 1;
        loop_rate.sleep();
    }

    delete relay;
    return 0;
}
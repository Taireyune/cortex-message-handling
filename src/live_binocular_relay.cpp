#include "cortex_message_handling/blackfly_binocular_relay.h"

int main(int argc, char **argv)
{
    // ros init
    ros::init(argc, argv, "Live_binocular_relay");
    
    // find camera
    Blackfly_camera_finder finder;

    // initialize nodes and camera
    // Blackfly_binocular_relay relay(finder.get_camera(0));

    int rate = 60;

    // relay.run_singlethread(rate);

    // // cleanup
    // relay.release_cameras();

    string suffix = "_1";
    Blackfly_video_relay camera(finder.get_camera(0), suffix);
    camera.start_acquisition(rate);
    // thread trigger_thread(triggers, rate);
    // sleep(1);
    // thread relay_thread(camera.start_acquisition, rate);

    return 0;   
}

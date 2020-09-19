#include "cortex_message_handling/blackfly_binocular_relay.h"

int main(int argc, char **argv)
{
    // ros init
    ros::init(argc, argv, "Live_binocular_relay");
    
    // find camera
    Blackfly_camera_finder finder;

    // initialize nodes and camera
    Blackfly_binocular_relay relay(finder.get_camera(0));

    int rate = 55;

    relay.run_singlethread(rate);

    // cleanup
    relay.release_cameras();
    return 0;
}

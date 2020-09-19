#include "cortex_message_handling/blackfly_binocular_relay.h"

int main(int argc, char **argv)
{
    // ros init
    ros::init(argc, argv, "Live_binocular_relay");
    
    // initialize nodes and camera
    Blackfly_binocular_relay relay;

    int rate = 40;

    relay.run_multithread(rate);
    relay.release_cameras();

    return 0;
}

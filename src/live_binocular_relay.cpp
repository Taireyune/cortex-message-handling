#include "cortex_message_handling/blackfly_binocular_relay.h"

int main(int argc, char **argv)
{
    // ros init
    ros::init(argc, argv, "Live_binocular_relay");
    
    // initialize nodes and camera
    Blackfly_binocular_relay relay;
    
    ros::Rate loop_rate(60);

    int count = 0;
    while (true)
    {
        cout << "[live_binocular_relay.main] loop count: " << count << endl;
        relay.update_attributes();
        count++;

        // for opencv
        if (cv::waitKey(10) == 27)
        {
            cout << "[live_binocular_relay.main] exit command." << endl;
            break;
        }
        loop_rate.sleep();
    }
    relay.release_cameras();

    return 0;
}

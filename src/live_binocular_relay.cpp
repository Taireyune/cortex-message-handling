#include "cortex_message_handling/blackfly_binocular_relay.h"

int main(int argc, char **argv)
{
    // ros init
    ros::init(argc, argv, "Live_binocular_relay");
    
    // initialize nodes and camera
    Blackfly_binocular_relay* relay = new Blackfly_binocular_relay();
    
    // set ros loop rate
    const int rate = 60;
    ros::Rate loop_rate(rate);

    // timer
    chrono::steady_clock::time_point time_now = chrono::steady_clock::now();

    int count = 0;
    // while (ros::ok())
    while (count < 300)
    {
        // timer
        cout << "Framerate = " 
            << 1000000.0 / chrono::duration_cast<chrono::microseconds>(chrono::steady_clock::now() - time_now).count() 
            << " hz" << endl;
        time_now = chrono::steady_clock::now();

        // cout << "[live_binocular_relay.main] loop count: " << count << endl;
        relay->update_attributes();
        count++;

        if (count % rate == 0)
        {
            cout << "[live_binocular_relay.main] loop count: " << count << endl;
            // cout << "[live_binocular_relay.main] frame rate: " << time_now << endl;
        }

        // for opencv
        if (cv::waitKey(10) == 27)
        {
            cout << "[live_binocular_relay.main] exit command." << endl;
            break;
        }
        loop_rate.sleep();
    }
    relay->release_cameras();

    return 0;
}

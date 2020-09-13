#ifndef SIMULATOR_RELAY
#define SIMULATOR_RELAY

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include "cortex_message_handling/CortexCommands.h"
#include "cortex_message_handling/BinocularFeed.h"

struct Cortex_commands
{
    int32_t waist           = 0;
    int32_t shoulder        = 0;
    int32_t elbow           = 0;
    int32_t wrist_angle     = 0;
    int32_t wrist_rotate    = 0;
    int32_t gripper         = 0;
};

class Simulator_relay
{
    
public:
    const int identifier;
    Simulator_relay(const int Identifier);
    ~Simulator_relay();

    // methods
    Cortex_commands update_attributes(int32_t testing);

private:
    // cortex commands attributes
    Cortex_commands cortex_commands;

    // ros objects
    ros::NodeHandle n;
    ros::Subscriber cortex_commands_sub;
    ros::Publisher joint_states_pub;
    ros::Publisher video_feed_pub;

    // methods
    void command_callback(const cortex_message_handling::CortexCommands &msg);
};

#define MODULE_API

extern "C" {
    MODULE_API void ros_init(const int identifier);
    MODULE_API Simulator_relay* init_simulator_relay(const int Identifier);
    MODULE_API void destroy_simulator_relay(Simulator_relay* relay);
    MODULE_API Cortex_commands update_simulator_attributes(Simulator_relay* relay, int testing);
}

#endif

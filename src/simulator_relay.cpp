#include "cortex_message_handling/simulator_relay.h"

Simulator_relay::Simulator_relay(const int Identifier)
    : identifier(Identifier)
{  
    // subscribes to commands for robot movement
    std::string cortex_command_identifier = "px150/cortex_commands_S" + std::to_string(identifier);
    cortex_commands_sub = n.subscribe(cortex_command_identifier, 1, &Simulator_relay::command_callback, this);

    // joint state publisher
    std::string joint_states_identifier = "px150/joint_states_S" + std::to_string(identifier);
    joint_states_pub = n.advertise<sensor_msgs::JointState>(joint_states_identifier, 1);

    // binocular feed publisher
    std::string binocular_feed_identifier = "px150/binocular_feed_S" + std::to_string(identifier);
    video_feed_pub = n.advertise<cortex_message_handling::BinocularFeed>(binocular_feed_identifier, 1);

    // std::cout << "[Simulator_relay " << identifier << "] relay object instantiated \n";
    ROS_INFO("[Simulator_relay %i] relay object instantiated", identifier);
}

Simulator_relay::~Simulator_relay()
{
}

void Simulator_relay::command_callback(const cortex_message_handling::CortexCommands &msg)
{
    cortex_commands.waist           = msg.waist;
    cortex_commands.shoulder        = msg.shoulder;
    cortex_commands.elbow           = msg.elbow;
    cortex_commands.wrist_angle     = msg.wrist_angle;
    cortex_commands.wrist_rotate    = msg.wrist_rotate;
    cortex_commands.gripper         = msg.gripper;
}

Cortex_commands Simulator_relay::update_attributes(int32_t testing)
{
    // video and joint state msg class
    sensor_msgs::JointState joint_states; 
    cortex_message_handling::BinocularFeed binocular_feed;

    // binocular feed message
    binocular_feed.testing = testing; 
    video_feed_pub.publish(binocular_feed);

    // std::cout << "[Simulator_relay::update_attributes " << identifier << "] testing: " << testing << "\n";
    ROS_INFO("[Simulator_relay::update_attributes %i] testing: %i", identifier, testing);

    // joint state message
    joint_states.position = {(double)testing, 0, 0, 0};
    joint_states_pub.publish(joint_states);

    // ROS_INFO("[Simulator_relay::update_attributes " + identifier + "] testing: %i", testing);
    ros::spinOnce();

    return cortex_commands;
}

MODULE_API void ros_init(const int identifier)
{
    // not sure how to properly create **argv works
    int argc = 1;
    char name[20] = "./simulator_relay";
    char *name_p = &name[0];
    char **argv;
    argv = &name_p;
    std::string simulator_relay_identifier = "simulator_relay_S" + std::to_string(identifier);
    ros::init(argc, argv, simulator_relay_identifier);
    ROS_INFO("[ros_init] ros::init() complete");
}

MODULE_API Simulator_relay* init_simulator_relay(const int Identifier)
{
    return new Simulator_relay(Identifier);
}

MODULE_API void destroy_simulator_relay(Simulator_relay* relay)
{
    delete relay;
}

MODULE_API Cortex_commands update_simulator_attributes(Simulator_relay* relay, int32_t testing)
{
    return relay->update_attributes(testing);
}

#include "cortex_message_handling/simulator_relay.h"
#include "dlfcn.h"

int main(int argc, char **argv)
{
    using std::cout;
    using std::cerr;

    cout << "[relay_debug] Opening libsimulator_relay.so...\n";
    void* lib = dlopen("libsimulator_relay.so", RTLD_LAZY);

    if (!lib) {
        cerr << "[relay_debug] Cannot open library: " << dlerror() << '\n';
        return 1;
    }
    dlerror();
    cout << "[relay_debug] Opening libsimulator_relay.so successful.\n";


    // ros_init
    typedef void (*ros_init_t)(const int);
    ros_init_t ros_init = (ros_init_t) dlsym(lib, "ros_init");
    char *dlsym_error = dlerror();
    if (dlsym_error) {
        cerr << "[relay_debug] Cannot load symbol 'ros_init': " << dlsym_error <<
            '\n';
        dlclose(lib);
        return 1;
    }
    ros_init(1);


    // simulator_relay
    typedef void* (*init_simulator_relay_t)(int);

    // load the symbols
    cout << "[relay_debug] Loading symbols...\n";
    init_simulator_relay_t init_simulator_relay = (init_simulator_relay_t) dlsym(lib, "init_simulator_relay");
    dlsym_error = dlerror();
    if (dlsym_error) {
        cerr << "[relay_debug] Cannot load symbol 'init_simulator_relay': " << dlsym_error <<
            '\n';
        dlclose(lib);
        return 1;
    }

    cout << "[relay_debug] Calling init_simulator_relay...\n";
    void* relay = init_simulator_relay(1);
    
    typedef void (*destroy_simulator_relay_t)(void*);
    typedef Cortex_commands (*update_simulator_attributes_t)(void*, int);

    destroy_simulator_relay_t destroy_simulator_relay = (destroy_simulator_relay_t) dlsym(lib, "destroy_simulator_relay");
    dlsym_error = dlerror();
    if (dlsym_error) {
        cerr << "[relay_debug] Cannot load symbol 'destroy_simulator_relay': " << dlsym_error <<
            '\n';
        dlclose(lib);
        return 1;
    }

    update_simulator_attributes_t update_simulator_attributes = (update_simulator_attributes_t) dlsym(lib, "update_simulator_attributes");
    dlsym_error = dlerror();
    if (dlsym_error) {
        cerr << "[relay_debug] Cannot load symbol 'update_simulator_attributes': " << dlsym_error <<
            '\n';
        dlclose(lib);
        return 1;
    }
    ros::Rate loop_rate(10);
    int testing = 0;
    Cortex_commands cortex_commands;

    while (testing < 100)
    {
        cout << "[relay_debug] inloop testing: " << testing << "\n";
        cortex_commands = update_simulator_attributes(relay, testing);
        cout << "waist value is: " << cortex_commands.waist << '\n';
        testing += 1;
        loop_rate.sleep();
    }

    destroy_simulator_relay(relay);
    return 0;
}
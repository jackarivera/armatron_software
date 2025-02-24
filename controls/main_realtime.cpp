#include "real_time_daemon.hpp"
#include "can_handler.hpp"
#include "robot_interface.hpp"
#include <iostream>

int main()
{
    try {
        // Bring up can0 externally:
        // sudo ip link set can0 type can bitrate 500000
        // sudo ip link set can0 up

        // 1) Create the CAN handler
        CANHandler can("can0");

        // 2) RobotInterface with up to 7 motors
        RobotInterface robot(can);

        // 3) RealTimeDaemon
        RealTimeDaemon daemon(robot);
        daemon.start();

        // Wait until Ctrl+C or kill
        std::cout << "[main_realtime] Running. Press Ctrl+C to exit.\n";
        while(true) {
            std::this_thread::sleep_for(std::chrono::seconds(10));
        }

        // on shutdown
        daemon.stop();
    } 
    catch (std::exception &ex) {
        std::cerr << "Exception: " << ex.what() << std::endl;
        return 1;
    }
    return 0;
}

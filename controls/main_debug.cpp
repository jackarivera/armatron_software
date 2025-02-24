#include "can_handler.hpp"
#include "motor_interface.hpp"
#include "robot_interface.hpp"
#include <iostream>
#include <chrono>
#include <thread>

int main()
{
    try {
        // 1) Bring up can0 externally: 
        //    sudo ip link set can0 type can bitrate 500000
        //    sudo ip link set can0 up

        // 2) Create CAN interface
        std::cerr << "Before creating CANHandler...\n";
        CANHandler can("can0");
        std::cerr << "After creating CANHandler.\n";

        // 3) Create a RobotInterface for 7 motors
        std::cerr << "Before creating robot...\n";
        RobotInterface robot(can);
        std::cerr << "After creating robot...\n";

        // 4) Let's just test motor #1 for now
        std::cerr << "Before getMotor...\n";
        auto& motor1 = robot.getMotor(1);
        std::cerr << "After getMotor...\n";

        // Turn motor on
        std::cerr << "Before motorOn...\n";
        motor1.motorOn();
        std::cerr << "After motorOn...\n";

        // Wait a moment
        //std::this_thread::sleep_for(std::chrono::milliseconds(200));

        // Possibly do a torque command 
        // Let's set torque = 500 in raw units => ~ ~4A 
        // motor1.setTorque(500);

        // Read the state to see if we got a response
        while(true){
        motor1.readState2();
        const auto st = motor1.getState();

        std::cerr << "[Main] Motor1 => Temp=" << st.temperatureC 
                  << "C, TorqueCurrent=" << st.torqueCurrentA
                  << "A, Speed=" << st.speedDeg_s << " deg/s\n";
        }

        // Turn off
        motor1.motorOff();

        std::cout << "[Main] Done.\n";
    }
    catch(const std::exception &ex) {
        std::cerr << "[Main] Exception: " << ex.what() << std::endl;
        return 1;
    }
    return 0;
}

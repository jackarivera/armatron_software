#include "can_handler.hpp"
#include "motor_interface.hpp"
#include "robot_interface.hpp"
#include <iostream>
#include <memory>
#include <thread>
#include <chrono>

int main()
{
    try {
        // Create CAN handler on interface 'can0'
        CANHandler can("can0");

        // Create global RobotInterface with 7 motors
        RobotInterface robot(can);

        // Example: Configure motor #1 to be in torque mode
        auto& motor1 = robot.getMotor(1);
        motor1.setControlMode(ControlMode::Torque);
        motor1.setDesiredTorque(5.0); // 5 N·m

        // Example: motor #2 in velocity mode
        auto& motor2 = robot.getMotor(2);
        motor2.setControlMode(ControlMode::Velocity);
        motor2.setDesiredVelocity(3.14); // ~1 rotation/s ( rad/s )

        // Example: motor #3 in position mode
        auto& motor3 = robot.getMotor(3);
        motor3.setControlMode(ControlMode::Position);
        motor3.setDesiredPosition(1.57); // ~90 deg

        // We'll do a simple loop simulating a control cycle
        for (int cycle = 0; cycle < 10; ++cycle)
        {
            robot.updateAll(); // read state from each motor, handle errors, send commands
            std::this_thread::sleep_for(std::chrono::milliseconds(10)); // 100 Hz
        }

        // Suppose we shut down motors
        for (int i = 1; i <= 7; ++i) {
            robot.getMotor(i).clearErrors();
            robot.getMotor(i).setControlMode(ControlMode::Torque);
            robot.getMotor(i).setDesiredTorque(0.0);
            // one last update to ensure torque=0
        }
        robot.updateAll();

        std::cout << "Exiting main.\n";
    }
    catch (const std::exception& ex) {
        std::cerr << "Exception in main: " << ex.what() << std::endl;
        return 1;
    }
    return 0;
}

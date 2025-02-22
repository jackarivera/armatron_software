#ifndef ROBOT_INTERFACE_HPP
#define ROBOT_INTERFACE_HPP

#include "motor_interface.hpp"
#include <vector>

/**
 * @brief RobotInterface manages multiple MG motors.
 *        It can easily scale from 1 to 7 or more joints.
 */
class RobotInterface
{
public:
    /**
     * @param canRef A reference to an already-initialized CANHandler
     * @param numMotors number of motors in your system, default 7
     */
    RobotInterface(CANHandler& canRef);

    /**
     * @brief Get a reference to motor i [1..numMotors].
     */
    Motor& getMotor(int i);

    /**
     * @brief Example update: read each motor's state for logging or safety checks.
     */
    void updateAll();

private:
    std::vector<Motor> m_motors;
};

#endif // ROBOT_INTERFACE_HPP

#ifndef ROBOT_INTERFACE_HPP
#define ROBOT_INTERFACE_HPP

#include "motor_interface.hpp"
#include <vector>

/**
 * @brief A global RobotInterface that creates and owns 7 motors.
 *        Provides an update() method to read states and send commands each cycle.
 */
class RobotInterface
{
public:
    /**
     * @brief Constructor
     * @param canHandler A shared or reference to your already-initialized CANHandler
     */
    RobotInterface(CANHandler& canHandler);

    /**
     * @brief Retrieve reference to a specific motor [0..6].
     */
    Motor& getMotor(int jointIndex);

    /**
     * @brief Call once per control cycle. 
     *        Reads each motor’s state, checks for errors, and sends commands as needed.
     */
    void updateAll();

private:
    std::vector<Motor> m_motors;
};

#endif // ROBOT_INTERFACE_HPP

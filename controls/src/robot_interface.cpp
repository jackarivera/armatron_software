#include "robot_interface.hpp"
#include "motor_defs.hpp"
#include <stdexcept>

RobotInterface::RobotInterface(CANHandler& canRef)
{
    // Create 7 motors with IDs 0 through 6 - NOTE THE NEGATIVE 1's NEED TO BE CHANGED TO TORQUE CONSTANTS
    // Joint 0 - MG8015 - Base Shoulder
    m_motors.emplace_back(static_cast<uint8_t>(1), canRef, MG8015_REDUCTION_RATIO, MG8015_MAX_TORQUE_NM);
    // Joint 1 - MG8015 - Mid Shoulder
    // m_motors.emplace_back(static_cast<uint8_t>(2), canRef, MG8015_REDUCTION_RATIO, MG8015_MAX_TORQUE_NM);
    // // Joint 2 - MG8008 - Brachium Shoulder
    // m_motors.emplace_back(static_cast<uint8_t>(3), canRef, MG8008_REDUCTION_RATIO, MG8008_MAX_TORQUE_NM);
    // // Joint 3 - MG8008 - Elbow Flexor
    // m_motors.emplace_back(static_cast<uint8_t>(4), canRef, MG8008_REDUCTION_RATIO, MG8008_MAX_TORQUE_NM);
    // // Joint 4 - MG4010 - Forearm Rotator
    // m_motors.emplace_back(static_cast<uint8_t>(5), canRef, MG4010_REDUCTION_RATIO, MG4010_MAX_TORQUE_NM);
    // // Joint 5 - MG4005 - Wrist Differential #1 
    // m_motors.emplace_back(static_cast<uint8_t>(6), canRef, MG4005_REDUCTION_RATIO, MG4005_MAX_TORQUE_NM);
    // // Joint 6 - MG4005 - Wrist Differential #2
    // m_motors.emplace_back(static_cast<uint8_t>(7), canRef, MG4005_REDUCTION_RATIO, MG4005_MAX_TORQUE_NM);
}

Motor& RobotInterface::getMotor(int i)
{
    if (i < 1 || i > (int)m_motors.size()) {
        throw std::out_of_range("Motor index out of range");
    }
    return m_motors[i - 1];
}

void RobotInterface::updateAll()
{
    for(auto &m : m_motors) {
        // For demonstration, let's do readState2
        m.readState2();
    }
}

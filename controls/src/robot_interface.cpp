#include "robot_interface.hpp"
#include "motor_defs.hpp"
#include <stdexcept>

RobotInterface::RobotInterface(CANHandler& canRef)
{
    // Create 7 motors with IDs 0 through 6 - NOTE THE NEGATIVE 1's NEED TO BE CHANGED TO TORQUE CONSTANTS
    // Joint 1 - MG8015 - Base Shoulder
    m_motors.emplace_back(static_cast<uint8_t>(1), canRef, MG8015_SINGLE_TURN_DEG_SCALE_MAX, JOINT_1_NM_TO_IQ_M, JOINT_1_NM_TO_IQ_B, JOINT_1_ANGLE_LIMIT_LOW, JOINT_1_ANGLE_LIMIT_HIGH, false);
    // Joint 2 - MG8015 - Mid Shoulder
    m_motors.emplace_back(static_cast<uint8_t>(2), canRef, MG8015_SINGLE_TURN_DEG_SCALE_MAX, JOINT_2_NM_TO_IQ_M, JOINT_2_NM_TO_IQ_B, JOINT_2_ANGLE_LIMIT_LOW, JOINT_2_ANGLE_LIMIT_HIGH, false);
    // // Joint 3 - MG8008 - Brachium Shoulder
    // m_motors.emplace_back(static_cast<uint8_t>(3), canRef, MG8008_SINGLE_TURN_DEG_SCALE_MAX, JOINT_3_NM_TO_IQ_M, JOINT_3_NM_TO_IQ_B, JOINT_3_ANGLE_LIMIT_LOW, JOINT_3_ANGLE_LIMIT_HIGH, false);
    // // Joint 4 - MG8008 - Elbow Flexor
    // m_motors.emplace_back(static_cast<uint8_t>(4), canRef, MG8008_SINGLE_TURN_DEG_SCALE_MAX, JOINT_4_NM_TO_IQ_M, JOINT_4_NM_TO_IQ_B, JOINT_4_ANGLE_LIMIT_LOW, JOINT_4_ANGLE_LIMIT_HIGH, false);
    // // Joint 5 - MG4010 - Forearm Rotator
    // m_motors.emplace_back(static_cast<uint8_t>(5), canRef, MG4010_SINGLE_TURN_DEG_SCALE_MAX, JOINT_5_NM_TO_IQ_M, JOINT_5_NM_TO_IQ_B, JOINT_5_ANGLE_LIMIT_LOW, JOINT_5_ANGLE_LIMIT_HIGH, false);
    // // Joint 6 - MG4005 - Wrist Differential #1 
    // m_motors.emplace_back(static_cast<uint8_t>(6), canRef, MG4005_SINGLE_TURN_DEG_SCALE_MAX, JOINT_6_NM_TO_IQ_M, JOINT_6_NM_TO_IQ_B, JOINT_6_ANGLE_LIMIT_LOW, JOINT_6_ANGLE_LIMIT_HIGH, true);
    // // Joint 7 - MG4005 - Wrist Differential #2
    // m_motors.emplace_back(static_cast<uint8_t>(7), canRef, MG4005_SINGLE_TURN_DEG_SCALE_MAX, JOINT_7_NM_TO_IQ_M, JOINT_7_NM_TO_IQ_B, JOINT_7_ANGLE_LIMIT_LOW, JOINT_7_ANGLE_LIMIT_HIGH, true);
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
        m.readSingleAngle();
    }
}

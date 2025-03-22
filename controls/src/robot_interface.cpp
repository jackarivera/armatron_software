#include "robot_interface.hpp"
#include "motor_defs.hpp"
#include <stdexcept>
#include <iostream>

RobotInterface::RobotInterface(CANHandler& canRef)
{
    // Create 7 motors with IDs 0 through 6 - NOTE THE NEGATIVE 1's NEED TO BE CHANGED TO TORQUE CONSTANTS
    // Joint 1 - MG8015 - Base Shoulder
    m_motors.emplace_back(static_cast<uint8_t>(1), canRef, MG8015_SINGLE_TURN_DEG_SCALE_MAX, JOINT_1_NM_TO_IQ_M, JOINT_1_NM_TO_IQ_B, JOINT_1_ANGLE_LIMIT_LOW, JOINT_1_ANGLE_LIMIT_HIGH, false);
    // Joint 2 - MG8015 - Mid Shoulder
    m_motors.emplace_back(static_cast<uint8_t>(2), canRef, MG8015_SINGLE_TURN_DEG_SCALE_MAX, JOINT_2_NM_TO_IQ_M, JOINT_2_NM_TO_IQ_B, JOINT_2_ANGLE_LIMIT_LOW, JOINT_2_ANGLE_LIMIT_HIGH, false);
    // // Joint 3 - MG8008 - Brachium Shoulder
    m_motors.emplace_back(static_cast<uint8_t>(3), canRef, MG8008_SINGLE_TURN_DEG_SCALE_MAX, JOINT_3_NM_TO_IQ_M, JOINT_3_NM_TO_IQ_B, JOINT_3_ANGLE_LIMIT_LOW, JOINT_3_ANGLE_LIMIT_HIGH, false);
    // // Joint 4 - MG8008 - Elbow Flexor
    m_motors.emplace_back(static_cast<uint8_t>(4), canRef, MG8008_SINGLE_TURN_DEG_SCALE_MAX, JOINT_4_NM_TO_IQ_M, JOINT_4_NM_TO_IQ_B, JOINT_4_ANGLE_LIMIT_LOW, JOINT_4_ANGLE_LIMIT_HIGH, false);
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
        m.readMultiAngle();
    }
}

// Sets joint angles for motors 1 through n - input angles in radians (they are converted to raw units after)
void RobotInterface::setMultiJointAngles(std::vector<float> joint_angles, std::vector<float> joint_speeds) {
    for (int i = 1; i <= joint_angles.size(); i++){
        float ang_target_rad = joint_angles.at(i-1);
        std::cout << "[RobotInterface] Multi Joint Command Received | Motor: " << i << " | Target (rad): " << ang_target_rad << "\n";
        if (ang_target_rad >= 0.0) {
            auto &curr_mot = m_motors[i-1];
            float ang_target_raw = curr_mot.motorRadiansToRaw(joint_angles.at(i-1));
            float ang_curr_raw = curr_mot.getState().positionDeg;
            float maxSpeed = joint_speeds.at(i-1);
            int spinDir = (ang_target_raw - ang_curr_raw) < 0 ? 1 : 0;
            m_motors[i-1].setMultiAngleWithSpeed(static_cast<int32_t>(ang_target_raw), static_cast<uint16_t>(maxSpeed));
            std::cout << "                 Target Angle Raw: " << ang_target_raw << "\n";
            std::cout << "                 Current Angle Raw: " << ang_curr_raw << "\n";
            std::cout << "                 MaxSpeed: " << maxSpeed << "\n";
            std::cout << "                 Spin Direction: " << spinDir << "\n";
        }
    }
}

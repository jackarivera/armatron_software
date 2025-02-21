#include "motor_interface.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace
{
    // For single motor commands, ID = 0x140 + m_motor_id
    // Example: motor_id=1 => 0x141
    inline int motorCommandID(uint8_t motorID)
    {
        return (0x140 + motorID);
    }

    // Example placeholders for scaling from N·m to the motor's integer torque command.
    // Each motor and driver might require different scaling based on doc specs.
    // You must adapt these carefully.
    inline int16_t torqueNmToProtocol(double nm)
    {
        // E.g., let's guess 16.5A ~ 2048 => 1 A => 124 steps => 1 N·m => ??? 
        // This is purely illustrative. 
        const double maxNm = 16.5; // example max torque
        if (nm > maxNm)  nm = maxNm;
        if (nm < -maxNm) nm = -maxNm;
        // map [-16.5..16.5] => [-2048..2048]
        double ratio = 2048.0 / maxNm;
        return static_cast<int16_t>(std::round(nm * ratio));
    }

    // For velocity, doc says speed is in 0.01 dps internally. 
    // We'll keep a placeholder conversion from rad/s -> 0.01 deg/s.
    inline int32_t radPerSecToProtocol(double rad_s)
    {
        // rad -> deg
        double deg_s = rad_s * (180.0 / M_PI);
        // times 100 for 0.01 deg/s
        double val = deg_s * 100.0;
        // clamp or just cast
        if (val > 2.147e9) val = 2.147e9;
        if (val < -2.147e9) val = -2.147e9;
        return static_cast<int32_t>(std::round(val));
    }

    // For position, doc says 0.01 deg increments in a 32-bit. 
    // So rad -> deg -> *100.
    inline int32_t radToProtocolPosition(double rad)
    {
        double deg = rad * (180.0 / M_PI);
        double val = deg * 100.0; 
        if (val > 2.147e9)  val = 2.147e9;
        if (val < -2.147e9) val = -2.147e9;
        return static_cast<int32_t>(std::round(val));
    }
}

Motor::Motor(uint8_t motor_id, CANHandler& canHandler, float gear_ratio, float torque_constant)
    : m_motor_id(motor_id)
    , m_gear_ratio(gear_ratio)
    , m_torque_constant(torque_constant)
    , m_can(canHandler)
    , m_mode(ControlMode::Position) // default
{
}

void Motor::setControlMode(ControlMode mode)
{
    m_mode = mode;
}

void Motor::setDesiredTorque(double torqueNm)
{
    // store in SI
    m_desiredTorqueNm = torqueNm;
}

void Motor::setDesiredVelocity(double velRad_s)
{
    m_desiredVelocityRad = velRad_s;
}

void Motor::setDesiredPosition(double posRad)
{
    m_desiredPositionRad = posRad;
}

void Motor::update()
{
    // 1) Read the current motor state => populate m_state with fresh data
    readStateFromMotor();

    // If an error is present, maybe we skip commanding? Or we handle it right away.
    if (m_state.errorPresent) {
        std::cerr << "[Motor] Error present on MotorID=" << (int)m_motor_id
                  << " code=" << m_state.errorCode << ". Attempting to handle...\n";
        // For skeleton, we do nothing else. 
        // You might auto-clear errors, or torque off, etc.
        return;
    }

    // 2) Send command based on the control mode
    switch(m_mode) {
    case ControlMode::Torque:
        sendTorqueCommand(m_desiredTorqueNm);
        break;
    case ControlMode::Velocity:
        sendVelocityCommand(m_desiredVelocityRad);
        break;
    case ControlMode::Position:
        sendPositionCommand(m_desiredPositionRad);
        break;
    default:
        break; 
    }
}

const MotorState& Motor::getState() const
{
    return m_state;
}

void Motor::clearErrors()
{
    // Command 0x9B: Clear error
    sendCmd(0x9B);
}

/* Private methods */

void Motor::readStateFromMotor()
{
    // Example: We can request the motor state with 0x9C => (Read motor state 2).
    // Then parse the reply in a loop until we find the frame for motor.
    // For skeleton, we'll do a single request/receive example.

    sendCmd(0x9C);

    // Attempt to read one frame:
    struct can_frame frame;
    if (m_can.receiveMessage(frame)) {
        // Typically, check if frame.can_id matches 0x140 + m_motor_id
        // Then parse frame.data to fill in m_state (temp, speed, etc.)
        // For brevity, we'll do placeholders:
        if ((frame.can_id & 0x7FF) == (0x140 + m_motor_id) && frame.can_dlc >= 8) {
            uint8_t cmdByte = frame.data[0];
            if (cmdByte == 0x9C) {
                // Suppose:
                // data layout:
                //  [0]=0x9C, [1]=temp, [2]=iqLow, [3]=iqHigh, [4]=speedLow, [5]=speedHigh, [6]=encLow, [7]=encHigh
                double temperature = static_cast<int8_t>(frame.data[1]);
                int16_t iqRaw = (static_cast<int16_t>(frame.data[3]) << 8) | (frame.data[2]);
                int16_t speedRaw = (static_cast<int16_t>(frame.data[5]) << 8) | (frame.data[4]);
                // fill out motor state
                m_state.temperatureC = temperature;
                // a rough mapping from iqRaw -> N·m
                m_state.torqueNm = (static_cast<double>(iqRaw) * (16.5 / 2048.0));
                // speed is in dps => convert to rad/s
                double deg_s = speedRaw * 1.0; // might be 1 dps, or 0.01 dps if doc says so
                m_state.velocityRad_s = deg_s * (M_PI / 180.0);

                m_state.errorPresent = false; 
                m_state.errorCode    = 0;
            }
        }
    }
}

void Motor::sendTorqueCommand(double nm)
{
    int16_t torque_cmd = torqueNmToProtocol(nm);
    // Command 0xA1 => torque
    // data => [ 0, 0, 0, torqueLow, torqueHigh, 0, 0 ]
    std::vector<uint8_t> data(7, 0);
    data[3] = static_cast<uint8_t>(torque_cmd & 0xFF);
    data[4] = static_cast<uint8_t>((torque_cmd >> 8) & 0xFF);

    sendCmd(0xA1, data);
}

void Motor::sendVelocityCommand(double rad_s)
{
    int32_t vel_cmd = radPerSecToProtocol(rad_s);
    // Command 0xA2 => speed
    std::vector<uint8_t> data(7, 0);
    data[3] = static_cast<uint8_t>( vel_cmd        & 0xFF);
    data[4] = static_cast<uint8_t>((vel_cmd >> 8)  & 0xFF);
    data[5] = static_cast<uint8_t>((vel_cmd >> 16) & 0xFF);
    data[6] = static_cast<uint8_t>((vel_cmd >> 24) & 0xFF);

    sendCmd(0xA2, data);
}

void Motor::sendPositionCommand(double rad)
{
    int32_t pos_cmd = radToProtocolPosition(rad);
    // Command 0xA3 => multi-loop angle
    std::vector<uint8_t> data(7, 0);
    data[3] = static_cast<uint8_t>( pos_cmd        & 0xFF);
    data[4] = static_cast<uint8_t>((pos_cmd >> 8)  & 0xFF);
    data[5] = static_cast<uint8_t>((pos_cmd >> 16) & 0xFF);
    data[6] = static_cast<uint8_t>((pos_cmd >> 24) & 0xFF);

    sendCmd(0xA3, data);
}

void Motor::sendCmd(uint8_t command, const std::vector<uint8_t>& data)
{
    int can_id = motorCommandID(m_motor_id);
    bool ok = m_can.sendMessage(can_id, command, data);
    if (!ok) {
        std::cerr << "[Motor] Failed to send command 0x"
                  << std::hex << (int)command
                  << " to motor ID=" << std::dec << (int)m_motor_id << "\n";
    }
}

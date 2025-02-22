#ifndef MOTOR_INTERFACE_HPP
#define MOTOR_INTERFACE_HPP

#include "can_handler.hpp"
#include <cstdint>
#include <vector>
#include <string>

/**
 * @brief Holds the latest known motor data for safety & logging.
 *        Updated whenever we read from the motor.
 */
struct MotorState
{
    double temperatureC    = 0.0;
    double busVoltage      = 0.0;
    double torqueCurrentA  = 0.0; ///< Actual current (like IQ)
    double speedDeg_s      = 0.0; ///< Speed in deg/s
    double positionDeg     = 0.0; ///< Single or multi-turn angle in degrees
    bool   errorPresent    = false;
    uint8_t errorCode      = 0;
};

/**
 * @brief A class implementing all commands from MG motor doc V2.35:
 *        - 0x80..0x81..0x88 for on/off/stop
 *        - 0xA0..0xA8 for open loop, torque, speed, angle
 *        - 0x30..0x34 for PID & accel
 *        - 0x90..0x95 for encoder, angle
 *        - 0x9A..0x9D for states & errors
 */
class Motor
{
public:
    /**
     * @param motorId  The motor's assigned ID on the bus (1..32)
     * @param canRef   Reference to a CANHandler. 
     * @param reduction_ratio Motor reduction ratio (reduction_ratio:1)
     * @param max_torque Max torque of each motor in Nm
     */
    Motor(uint8_t motorId, CANHandler& canRef, float reduction_ratio, float max_torque);

    /**
     * @brief Retrieve the last known motor state (populated from read ops).
     */
    const MotorState& getState() const;

    /**
     * @brief Turn motor OFF (0x80), clearing any prior commands.
     */
    void motorOff();

    /**
     * @brief Turn motor ON (0x88).
     */
    void motorOn();

    /**
     * @brief Motor Stop (0x81). Does not clear state, can be resumed.
     */
    void motorStop();

    /**
     * @brief Open-loop control (0xA0) - only valid for MS-series, but doc shows it in MF/MG too.
     *        Range: -850..850. 
     */
    void openLoopControl(int16_t powerControl);

    /**
     * @brief Torque closed-loop (0xA1). Range -2048..2048 => ~ +/- 16.5A on MG 
     */
    void setTorque(int16_t iqControl);

    /**
     * @brief Speed closed-loop (0xA2). 4-byte speed in 0.01 deg/s. 
     *        We'll accept an integer speed in 0.01 deg/s for convenience.
     */
    void setSpeed(int32_t speedControl);

    /**
     * @brief Multi-loop angle control 1 (0xA3). 4-byte angle in 0.01 deg.
     */
    void setMultiAngle(int32_t angleControl);

    /**
     * @brief Multi-loop angle control 2 (0xA4). angle + speed limit
     */
    void setMultiAngleWithSpeed(int32_t angle, uint16_t maxSpeed);

    /**
     * @brief Single-loop angle control 1 (0xA5). 
     *        spinDirection=0 => CW, 1 => CCW
     */
    void setSingleAngle(uint8_t spinDirection, uint16_t angle);

    /**
     * @brief Single-loop angle control 2 (0xA6). angle + speed limit
     */
    void setSingleAngleWithSpeed(uint8_t spinDirection, uint16_t angle, uint16_t maxSpeed);

    /**
     * @brief Increment angle control 1 (0xA7). 4-byte increment in 0.01 deg
     */
    void setIncrementAngle(int32_t incAngle);

    /**
     * @brief Increment angle control 2 (0xA8). incAngle + maxSpeed
     */
    void setIncrementAngleWithSpeed(int32_t incAngle, uint16_t maxSpeed);

    /**
     * @brief Read PID (0x30). Returns array of 6 param bytes: [AngKp,AngKi,SpdKp,SpdKi,TrqKp,TrqKi]
     */
    std::vector<uint8_t> readPID();

    /**
     * @brief Write PID to RAM (0x31). Not persistent after power-off.
     */
    void writePID_RAM(uint8_t angKp, uint8_t angKi, uint8_t spdKp, uint8_t spdKi, uint8_t iqKp, uint8_t iqKi);

    /**
     * @brief Write PID to ROM (0x32). Persists after power cycle.
     */
    void writePID_ROM(uint8_t angKp, uint8_t angKi, uint8_t spdKp, uint8_t spdKi, uint8_t iqKp, uint8_t iqKi);

    /**
     * @brief Read acceleration (0x33). 4-byte in 1 dps^2 
     */
    int32_t readAcceleration();

    /**
     * @brief Write acceleration to RAM (0x34). 
     */
    void writeAcceleration(int32_t accel);

    /**
     * @brief Read encoder (0x90). Fills motorState with (encoder, rawEncoder, offset).
     */
    void readEncoder();

    /**
     * @brief Write encoder offset to ROM (0x91). 14-bit offset, e.g. 0..16383 for MF. 
     */
    void writeEncoderOffset(uint16_t offset);

    /**
     * @brief Write current pos to ROM as zero (0x19). Takes effect after reset. 
     *        The doc says multiple writes can degrade chip life. Use sparingly.
     */
    void writeCurrentPosAsZero();

    /**
     * @brief Read multi-turn angle (0x92). 64-bit in 0.01 deg -> fill in state.
     */
    void readMultiAngle();

    /**
     * @brief Read single-turn angle (0x94). 32-bit in 0.01 deg -> fill in state.
     */
    void readSingleAngle();

    /**
     * @brief Clear motor angle (0x95). Clears multi & single turn data in RAM.
     */
    void clearAngle();

    /**
     * @brief Read motor state 1 & error (0x9A). Fills temp, voltage, error.
     */
    void readState1_Error();

    /**
     * @brief Clear motor error (0x9B).
     */
    void clearError();

    /**
     * @brief Read motor state 2 (0x9C). Fills temp, torque current, speed, encoder pos.
     */
    void readState2();

    /**
     * @brief Read motor state 3 (0x9D). Fills temp & phase currents (A/B/C).
     */
    void readState3();

private:
    uint8_t    m_motorId;
    CANHandler &m_can;
    MotorState m_state;
    float m_reduction_ratio  = 0.0;
    float m_max_torque       = 0.0;
    float m_rated_torque     = 0.0;
    float m_torque_constant  = 0.0;

    /**
     * @brief Helper: Single motor ID = 0x140 + m_motorId
     */
    int canID() const { return (0x140 + m_motorId); }

    /**
     * @brief Low-level send
     */
    bool sendCmd(uint8_t command, const std::vector<uint8_t>& data = {});

    /**
     * @brief Low-level receive & parse. For each command, we parse the 
     *        doc-defined response. Updates m_state. 
     *        This is done once per command in a blocking or single-shot approach.
     */
    void readFrameForCommand(uint8_t expectedCmd);
};

#endif // MOTOR_INTERFACE_HPP

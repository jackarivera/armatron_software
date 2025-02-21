#ifndef MOTOR_INTERFACE_HPP
#define MOTOR_INTERFACE_HPP

#include "can_handler.hpp"
#include <cstdint>
#include <vector>
#include <string>

/**
 * @brief Control modes for the motor.
 */
enum class ControlMode
{
    Torque,   // Control torque in N·m
    Velocity, // Control velocity in rad/s
    Position  // Control position in radians (single-loop angle implementation)
};

/**
 * @brief PID Parameters of the motor.
 */
 struct PIDParameters
 {
    uint8_t pos_kp;
    uint8_t pos_ki;
    uint8_t vel_kp;
    uint8_t vel_ki;
    uint8_t torque_kp;
    uint8_t torque_ki;
 };

/**
 * @brief Struct to hold the real-time state of a motor.
 *        For safety reasons, we store error flags, temperature, etc.
 */
struct MotorState
{
    double positionRad   = 0.0;   ///< Estimated position in radians
    double velocityRad_s = 0.0;   ///< Estimated velocity in rad/s
    double torqueNm      = 0.0;   ///< Estimated output torque in N·m
    double temperatureC  = 0.0;   ///< Motor temperature in °C
    double busVoltage    = 0.0;   ///< Motor bus voltage (optional read)
    bool   errorPresent  = false; ///< True if an error is flagged
    int    errorCode     = 0;     ///< Some code/bitmask
};

/**
 * @brief A MotorInterface that uses a CANHandler and MG Motor CANbus style protocol internally.
 */
class Motor
{
public:
    /**
     * @param motor_id   The assigned ID on the CAN bus (1..7).
     * @param canHandler Reference to an instantiated CANHandler.
     * @param gear_ratio Gear ratio-> Motor:Reducer (i.e. 9:1)
     * @param torque_constant Torque Consant in Nm/A
     */
    Motor(uint8_t motor_id, CANHandler& canHandler, float gear_ratio = 1.0f, float torque_constant = 1.0f);

    /**
     * @brief Set the control mode (Torque, Velocity, Position).
     */
    void setControlMode(ControlMode mode);

    /**
     * @brief Set a torque command in N·m (SI units).
     *        This value is used if controlMode == Torque.
     */
    void setDesiredTorque(double torqueNm);

    /**
     * @brief Set a velocity command in rad/s (SI units).
     *        This value is used if controlMode == Velocity.
     */
    void setDesiredVelocity(double velRad_s);

    /**
     * @brief Set a position command in radians (SI units).
     *        This value is used if controlMode == Position.
     */
    void setDesiredPosition(double posRad);

    /**
     * @brief Called each control cycle to:
     *        1) Read the motor state from the hardware (via CAN).
     *        2) If no error, send the relevant command (torque/vel/pos) else handle error 
     */
    void update();

    /**
     * @brief Sets the PID parameters of the motor. Writes to RAM
     */
     void writePIDParametersRAM();

    /**
    * @brief Sets the PID parameters of the motor. Writes to ROM (Permanently set after power shutdown)
    */
    void writePIDParametersROM();
    
    /**
     * @brief Reads the PID parameters of the motor by sending a 0x30 command
     */
     void readPIDParameters();

    /**
     * @brief Returns the last known motor state.
     */
    const MotorState& getState() const { return m_state; }

    /**
     * @brief Returns the motor ID.
     */
     const uint8_t getID() const { return m_motor_id; }

    /**
     * @brief Returns the motor gear ratio.
     */
     const uint8_t getGearRatio() const { return m_gear_ratio; }

    /**
     * @brief Returns the motor torque constant.
     */
     const uint8_t getTorqueConstant() const { return m_torque_constant; }

    /**
     * @brief Clears any error on the motor side (if recoverable).
     */
    void clearErrors();

private:
    uint8_t     m_motor_id;
    float       m_gear_ratio;
    float       m_torque_constant;
    CANHandler& m_can;
    ControlMode m_mode;

    // Desired setpoints (SI)
    double m_desiredTorqueNm    = 0.0;
    double m_desiredVelocityRad = 0.0;
    double m_desiredPositionRad = 0.0;

    // The real-time motor state.
    MotorState m_state;

    /**
     * @brief Perform a CAN read to update m_state.
     *        Skeleton example; actual parsing should decode frame data.
     */
    void readStateFromMotor();

    /**
     * @brief Send the commanded torque via the CAN protocol.
     *        Here, we do a simplistic mapping from N·m -> [protocol units].
     */
    void sendTorqueCommand(double nm);

    /**
     * @brief Send the commanded velocity in rad/s -> motor protocol units.
     */
    void sendVelocityCommand(double rad_s);

    /**
     * @brief Send the commanded position in radians -> motor protocol units.
     */
    void sendPositionCommand(double rad);

    /**
     * @brief Low-level function to send a raw command & data to this motor's ID.
     */
    void sendCmd(uint8_t command, const std::vector<uint8_t>& data = {});
};

#endif // MOTOR_INTERFACE_HPP

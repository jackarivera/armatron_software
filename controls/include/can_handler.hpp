#ifndef CAN_HANDLER_HPP
#define CAN_HANDLER_HPP

#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <net/if.h>
#include <string>
#include <vector>


/**
 * @brief Handles opening and managing a SocketCAN interface.
 *        Provides methods to send/receive raw CAN frames.
 */
class CANHandler
{
public:
    /**
     * @brief Constructor opens a raw CAN socket on the given interface.
     * @param interface_name e.g. \"can0\".
     */
    explicit CANHandler(const std::string& interface_name);

    /**
     * @brief Destructor closes the CAN socket.
     */
    ~CANHandler();

    /**
     * @brief Send a CAN frame with ID, command, and data bytes.
     * @param can_id  11-bit standard ID (e.g. 0x141 for motor 1).
     * @param command First data byte, from the protocol (0x80, 0xA1, etc.).
     * @param data    Additional data bytes.
     * @return True if successful, false otherwise.
     */
    bool sendMessage(int can_id, uint8_t command, const std::vector<uint8_t>& data);

    /**
     * @brief Read one CAN frame if available.
     * @param frame A reference to store the read frame.
     * @return True if a frame was read, false if read failed.
     */
    bool receiveMessage(struct can_frame& frame);

private:
    int m_socket_fd;
    struct sockaddr_can m_addr;
    struct ifreq m_ifr;
};

#endif // CAN_HANDLER_HPP

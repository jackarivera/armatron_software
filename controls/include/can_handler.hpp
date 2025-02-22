#ifndef CAN_HANDLER_HPP
#define CAN_HANDLER_HPP

#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <net/if.h>
#include <string>
#include <vector>

/**
 * @brief Manages SocketCAN communication for the MG motors. 
 *        Replaces the MCP-based approach from lkm_m5 with raw Linux sockets.
 */
class CANHandler
{
public:
    /**
     * @param interface_name  e.g., \"can0\" 
     * @throws std::runtime_error if socket fails to open/bind
     */
    explicit CANHandler(const std::string& interface_name);

    /**
     * @brief Closes socket upon destruction
     */
    ~CANHandler();

    /**
     * @brief Send a CAN frame (ID, command byte, plus data bytes).
     * @param can_id  Standard 11-bit ID, e.g. 0x141 for motor ID=1
     * @param command The first data byte from doc (0x88, 0x9C, etc.)
     * @param data    Additional data up to 7 bytes
     * @return True if write succeeded, false otherwise
     */
    bool sendMessage(int can_id, uint8_t command, const std::vector<uint8_t>& data);

    /**
     * @brief Attempt to read one CAN frame into 'frame'.
     * @return True if read a full frame successfully
     */
    bool receiveMessage(struct can_frame& frame);

private:
    int m_socket_fd;
    struct sockaddr_can m_addr;
    struct ifreq m_ifr;
};

#endif // CAN_HANDLER_HPP

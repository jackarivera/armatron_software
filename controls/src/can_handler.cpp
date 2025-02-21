#include "can_handler.hpp"
#include <stdexcept>    
#include <cstring>     
#include <iostream>    
#include <unistd.h>    
#include <sys/ioctl.h> 

CANHandler::CANHandler(const std::string& interface_name)
    : m_socket_fd(-1)
{
    // Create raw CAN socket
    m_socket_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (m_socket_fd < 0) {
        throw std::runtime_error("Failed to open CAN socket");
    }

    std::memset(&m_ifr, 0, sizeof(m_ifr));
    std::strncpy(m_ifr.ifr_name, interface_name.c_str(), IFNAMSIZ - 1);

    // Retrieve interface index
    if (ioctl(m_socket_fd, SIOCGIFINDEX, &m_ifr) < 0) {
        close(m_socket_fd);
        throw std::runtime_error("Failed to get interface index for CAN");
    }

    std::memset(&m_addr, 0, sizeof(m_addr));
    m_addr.can_family = AF_CAN;
    m_addr.can_ifindex = m_ifr.ifr_ifindex;

    // Bind socket
    if (bind(m_socket_fd, reinterpret_cast<struct sockaddr*>(&m_addr), sizeof(m_addr)) < 0) {
        close(m_socket_fd);
        throw std::runtime_error("Failed to bind CAN socket to interface");
    }
}

CANHandler::~CANHandler()
{
    if (m_socket_fd >= 0) {
        close(m_socket_fd);
    }
}

bool CANHandler::sendMessage(int can_id, uint8_t command, const std::vector<uint8_t>& data)
{
    if (m_socket_fd < 0) {
        std::cerr << "[CANHandler] Socket not open.\n";
        return false;
    }

    struct can_frame frame;
    std::memset(&frame, 0, sizeof(frame));

    frame.can_id = can_id & 0x7FF; // standard 11-bit
    // 1 command byte + data.size() must fit <= 8
    if (data.size() + 1 > 8) {
        std::cerr << "[CANHandler] Data size too large.\n";
        return false;
    }

    frame.can_dlc = data.size() + 1;
    frame.data[0] = command;
    for (size_t i = 0; i < data.size(); ++i) {
        frame.data[i + 1] = data[i];
    }

    ssize_t nbytes = write(m_socket_fd, &frame, sizeof(frame));
    return (nbytes == static_cast<ssize_t>(sizeof(frame)));
}

bool CANHandler::receiveMessage(struct can_frame& frame)
{
    if (m_socket_fd < 0) {
        std::cerr << "[CANHandler] Socket not open.\n";
        return false;
    }

    ssize_t nbytes = read(m_socket_fd, &frame, sizeof(frame));
    return (nbytes == static_cast<ssize_t>(sizeof(frame)));
}

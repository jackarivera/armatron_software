#include "can_handler.hpp"
#include <stdexcept>
#include <cstring>
#include <iostream>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include "utils.hpp"


CANHandler::CANHandler(const std::string& interface_name)
    : m_socket_fd(-1)
{
    // Create raw CAN socket
    m_socket_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    IFCANDEBUG(std::cout << "[CANHandler][DEBUG] Socket created: " << m_socket_fd << "\n");
    if (m_socket_fd < 0) {
        throw std::runtime_error("[CANHandler] Failed to open CAN socket");
    }

    std::memset(&m_ifr, 0, sizeof(m_ifr));
    std::strncpy(m_ifr.ifr_name, interface_name.c_str(), IFNAMSIZ - 1);

    // get interface index
    if (ioctl(m_socket_fd, SIOCGIFINDEX, &m_ifr) < 0) {
        close(m_socket_fd);
        throw std::runtime_error("[CANHandler] Failed to get IF index for " + interface_name);
    }
    IFCANDEBUG(std::cout << "[CANHandler][DEBUG] Interface index obtained: " << m_ifr.ifr_ifindex << "\n");

    std::memset(&m_addr, 0, sizeof(m_addr));
    m_addr.can_family = AF_CAN;
    m_addr.can_ifindex = m_ifr.ifr_ifindex;

    // bind socket
    if (bind(m_socket_fd, reinterpret_cast<struct sockaddr*>(&m_addr), sizeof(m_addr)) < 0) {
        close(m_socket_fd);
        throw std::runtime_error("[CANHandler] Failed to bind CAN socket to " + interface_name);
    }
    IFCANDEBUG(std::cout << "[CANHandler][DEBUG] Socket bound to interface " << interface_name << "\n");

    struct timeval tv;
    tv.tv_sec = 0;        // 0 seconds
    tv.tv_usec = 10000;   // 10 milliseconds timeout

    IFCANDEBUG(std::cout << "[CANHandler][DEBUG] Setting socket receive timeout\n");
    if (setsockopt(m_socket_fd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(tv)) < 0) {
        std::cerr << "[CANHandler][ERROR] Failed to set socket receive timeout.\n";
    }
}

CANHandler::~CANHandler()
{
    IFCANDEBUG(std::cout << "[CANHandler][DEBUG] Destructor called. Closing socket.\n");
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

    // Standard 11-bit ID
    frame.can_id = (can_id & 0x7FF);

    // We will always send 8 data bytes, matching typical MG motor commands 
    // which often show 8 bytes in the doc. Extra trailing bytes are zeros.
    frame.can_dlc = 8;

    // The first data byte is the command
    frame.data[0] = command;

    // Copy up to 7 bytes from 'data' into frame.data[1..7].
    const size_t maxCopy = 7; // since data[0] is command
    const size_t toCopy = (data.size() < maxCopy) ? data.size() : maxCopy;
    if (toCopy > 0) {
        std::memcpy(&frame.data[1], data.data(), toCopy);
    }
    IFCANDEBUG(
        std::cout << "[CANHandler][DEBUG] Sending CAN message: ID=" << (can_id & 0x7FF) 
                  << ", Command=0x" << std::hex << static_cast<int>(command) << std::dec
                  << ", Data=[";
        for (size_t i = 0; i < 7; ++i) {
            std::cout << "0x" << std::hex << static_cast<int>(frame.data[i+1]) << std::dec;
            if (i < 6) std::cout << ", ";
        }
        std::cout << "]\n"
    );

    // Write the frame
    ssize_t nbytes = write(m_socket_fd, &frame, sizeof(frame));
    IFCANDEBUG(std::cout << "[CANHandler][DEBUG] write() returned " << nbytes << "\n");
    return (nbytes == static_cast<ssize_t>(sizeof(frame)));
}


bool CANHandler::receiveMessage(struct can_frame& frame)
{
    if (m_socket_fd < 0) {
        std::cerr << "[CANHandler] Socket not open.\n";
        return false;
    }

    ssize_t nbytes = read(m_socket_fd, &frame, sizeof(frame));
    IFCANDEBUG(
        if (nbytes == static_cast<ssize_t>(sizeof(frame))) {
            std::cout << "[CANHandler][DEBUG] Received CAN message: ID=" << (frame.can_id & 0x7FF)
                      << ", Data=[";
            for (int i = 0; i < frame.can_dlc; ++i) {
                std::cout << "0x" << std::hex << static_cast<int>(frame.data[i]) << std::dec;
                if (i < frame.can_dlc - 1) std::cout << ", ";
            }
            std::cout << "]\n";
        } else {
            std::cout << "[CANHandler][DEBUG] read() returned " << nbytes << "\n";
        }
    );
    return (nbytes == static_cast<ssize_t>(sizeof(frame)));
}

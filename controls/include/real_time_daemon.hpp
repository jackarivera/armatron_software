#ifndef REAL_TIME_DAEMON_HPP
#define REAL_TIME_DAEMON_HPP

#include "robot_interface.hpp"
#include <string>
#include <thread>
#include <atomic>
#include <mutex>
#include <queue>

/**
 * @brief A minimal message structure to pass between Node and the real-time code
 */
struct IPCMessage {
    std::string json; 
};

/**
 * @brief RealTimeDaemon sets up:
 *  1) A high-frequency control loop for the motors
 *  2) A Unix domain socket server to receive commands from Node
 *  3) A mechanism to send motor states to Node
 */
class RealTimeDaemon
{
public:
    RealTimeDaemon(RobotInterface& robot);
    ~RealTimeDaemon();

    /**
     * @brief Start the daemon:
     *  1) Binds /tmp/robot_socket
     *  2) Spawns a real-time loop thread at high freq
     *  3) Spawns a socket listener thread
     */
    void start();

    /**
     * @brief Stop everything, join threads
     */
    void stop();

private:
    RobotInterface& m_robot;
    std::atomic<bool> m_running { false };

    // Socket stuff
    int m_sockfd;
    std::thread m_socketThread;
    std::string m_socketPath;

    // Real-time loop
    std::thread m_controlThread;

    // Thread-safe queue for inbound commands
    std::mutex m_inboundMutex;
    std::queue<IPCMessage> m_inboundQueue;

    // Outbound queuing 
    std::mutex m_outboundMutex;
    std::queue<IPCMessage> m_outboundQueue;

    void socketThreadFunc();
    void controlThreadFunc();

    // Helpers to parse JSON commands and update RobotInterface
    void handleCommand(const std::string& jsonStr);

    // For sending JSON messages back to Node
    void sendJson(const std::string& jsonStr);
};

#endif

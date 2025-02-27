#include "real_time_daemon.hpp"
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <stdexcept>
#include <iostream>
#include <chrono>
#include <cstring>
#include <sstream>
#include <fcntl.h>
#include <jsoncpp/json/json.h>
#include <algorithm>

#ifndef IFDEBUG
#ifdef DEBUG
#define IFDEBUG(x) do { x; } while (0);
#else
#define IFDEBUG(x) do {} while (0);
#endif
#endif


namespace
{
const char* DEFAULT_SOCKET_PATH = "/tmp/robot_socket";
}

/**********************************************************/
/* Constructor / Destructor                               */
/**********************************************************/
RealTimeDaemon::RealTimeDaemon(RobotInterface& robot)
    : m_robot(robot)
    , m_sockfd(-1)
    , m_socketPath(DEFAULT_SOCKET_PATH)
{
    // We might remove any stale socket file
    ::unlink(m_socketPath.c_str());
    IFDEBUG(std::cout << "[RealTimeDaemon][DEBUG] Constructor: Removed stale socket file if exists.\n")
}

RealTimeDaemon::~RealTimeDaemon()
{
    stop();
}

/**********************************************************/
/* Start / Stop                                           */
/**********************************************************/
void RealTimeDaemon::start()
{
    m_running = true;
    IFDEBUG(std::cout << "[RealTimeDaemon][DEBUG] Starting daemon.\n")

    // 1) Create the socket
    m_sockfd = socket(AF_UNIX, SOCK_STREAM, 0);
    IFDEBUG(std::cout << "[RealTimeDaemon][DEBUG] Socket created: " << m_sockfd << "\n")
    if (m_sockfd < 0) {
        throw std::runtime_error("Failed to create Unix domain socket");
    }

    // 2) Bind the socket
    sockaddr_un addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    std::strncpy(addr.sun_path, m_socketPath.c_str(), sizeof(addr.sun_path)-1);

    if (bind(m_sockfd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        close(m_sockfd);
        throw std::runtime_error("Failed to bind to " + m_socketPath);
    }
    IFDEBUG(std::cout << "[RealTimeDaemon][DEBUG] Socket bound to " << m_socketPath << "\n")

    // 3) Listen
    if (listen(m_sockfd, 5) < 0) {
        close(m_sockfd);
        throw std::runtime_error("listen() failed");
    }
    IFDEBUG(std::cout << "[RealTimeDaemon][DEBUG] Listening on socket.\n")

    // 4) Start the socket thread
    m_socketThread = std::thread(&RealTimeDaemon::socketThreadFunc, this);
    IFDEBUG(std::cout << "[RealTimeDaemon][DEBUG] Socket thread started.\n")

    // 5) Start the real-time control thread
    m_controlThread = std::thread(&RealTimeDaemon::controlThreadFunc, this);
    IFDEBUG(std::cout << "[RealTimeDaemon][DEBUG] Control thread started.\n")

    std::cout << "[RealTimeDaemon] Started, socket at " << m_socketPath << "\n";
}

void RealTimeDaemon::stop()
{
    if (!m_running) return;

    m_running = false;
    IFDEBUG(std::cout << "[RealTimeDaemon][DEBUG] Stopping daemon.\n")

    // close socket
    if (m_sockfd >= 0) {
        close(m_sockfd);
        m_sockfd = -1;
        IFDEBUG(std::cout << "[RealTimeDaemon][DEBUG] Socket closed.\n")
    }

    if (m_socketThread.joinable()) {
        m_socketThread.join();
        IFDEBUG(std::cout << "[RealTimeDaemon][DEBUG] Socket thread joined.\n")
    }
    if (m_controlThread.joinable()) {
        m_controlThread.join();
        IFDEBUG(std::cout << "[RealTimeDaemon][DEBUG] Control thread joined.\n")
    }

    // Close all connected client sockets.
    {
        std::lock_guard<std::mutex> lk(m_clientFdsMutex);
        for (auto fd : m_clientFds) {
            close(fd);
        }
        m_clientFds.clear();
    }
    ::unlink(m_socketPath.c_str());
    std::cout << "[RealTimeDaemon] Stopped.\n";
}

/**********************************************************/
/* Socket Thread                                          */
/**********************************************************/
void RealTimeDaemon::socketThreadFunc()
{
    while (m_running) {
        int clientFd = accept(m_sockfd, nullptr, nullptr);
        if (clientFd < 0) {
            if (!m_running) break;
            std::cerr << "[RealTimeDaemon] Accept error.\n";
            continue;
        }
        IFDEBUG(std::cout << "[RealTimeDaemon][DEBUG] Accepted client FD=" << clientFd << "\n");

        {
            // Add the client FD to our list.
            std::lock_guard<std::mutex> lk(m_clientFdsMutex);
            m_clientFds.push_back(clientFd);
        }
        // Spawn a new thread to handle incoming messages from this client.
        std::thread(&RealTimeDaemon::clientHandler, this, clientFd).detach();
    }
}

// This method handles reading from a single client connection.
void RealTimeDaemon::clientHandler(int clientFd)
{
    char buf[1024];
    std::stringstream partial;
    while (m_running) {
        ssize_t r = recv(clientFd, buf, sizeof(buf), 0);
        IFDEBUG(std::cout << "[RealTimeDaemon][DEBUG] Received " << r << " bytes from client FD=" << clientFd << "\n");
        if (r <= 0) {
            if (!partial.str().empty()) {
                std::string line = partial.str();
                std::lock_guard<std::mutex> lk(m_inboundMutex);
                m_inboundQueue.push( IPCMessage{ line } );
                IFDEBUG(std::cout << "[RealTimeDaemon][DEBUG] Flushed partial JSON: " << line << "\n");
            }
            break;
        }
        for (int i = 0; i < r; i++) {
            if (buf[i] == '\n') {
                std::string line = partial.str();
                partial.str(std::string());
                partial.clear();
                IFDEBUG(std::cout << "[RealTimeDaemon][DEBUG] Received JSON line: " << line << "\n");
                if (!line.empty()) {
                    std::lock_guard<std::mutex> lk(m_inboundMutex);
                    m_inboundQueue.push( IPCMessage{ line } );
                    IFDEBUG(std::cout << "[RealTimeDaemon][DEBUG] Pushed JSON line into queue: " << line << "\n");
                }
            } else {
                partial << buf[i];
            }
        }
    }
    close(clientFd);
    {
        std::lock_guard<std::mutex> lk(m_clientFdsMutex);
        auto it = std::find(m_clientFds.begin(), m_clientFds.end(), clientFd);
        if (it != m_clientFds.end()) {
            m_clientFds.erase(it);
        }
    }
    IFDEBUG(std::cout << "[RealTimeDaemon][DEBUG] Closed client FD=" << clientFd << "\n");
}


/**********************************************************/
/* Control Thread                                         */
/**********************************************************/
void RealTimeDaemon::controlThreadFunc()
{
    IFDEBUG(std::cout << "[RealTimeDaemon][DEBUG] Control thread running.\n")
    // 1 kHz loop
    auto nextTime = std::chrono::steady_clock::now();
    std::chrono::microseconds period(1000); 

    while (m_running) {
        // 1) Process inbound commands
        {
            std::lock_guard<std::mutex> lk(m_inboundMutex);
            while (!m_inboundQueue.empty()) {
                auto msg = m_inboundQueue.front();
                m_inboundQueue.pop();
                IFDEBUG(std::cout << "[RealTimeDaemon][DEBUG] Processing command: " << msg.json << "\n")
                handleCommand(msg.json);
            }
        }

        // 2) Do real-time update for all motors
        IFDEBUG(std::cout << "[RealTimeDaemon][DEBUG] Updating all motors.\n")
        m_robot.updateAll(); // This does CAN read/writes
        IFDEBUG(std::cout << "[RealTimeDaemon][DEBUG] Updated all motors.\n")

        // 3) Broadcast states at ~ 60Hz
        static int counter = 0;
        counter++;
        if (counter % (1000/60) == 0) {  // every ~ 16 ticks at 1 kHz => ~60 Hz
            Json::Value jroot;
            jroot["type"] = "motorStates";

            // gather each motor’s state
            for (int i = 1; i < 2; i++) {
                auto &mot = m_robot.getMotor(i);
                auto st   = mot.getState();
                Json::Value mjs;
                mjs["temp"]       = st.temperatureC;
                mjs["torqueA"]    = st.torqueCurrentA;
                mjs["speedDeg_s"] = st.speedDeg_s;
                mjs["posDeg"]     = st.positionDeg;
                mjs["error"]      = (st.errorPresent ? 1 : 0);
                jroot["motors"][std::to_string(i)] = mjs;
            }

            Json::StreamWriterBuilder builder;
            builder["indentation"] = ""; // Force compact, single-line output.
            std::string outStr = Json::writeString(builder, jroot);
            IFDEBUG(std::cout << "[RealTimeDaemon][DEBUG] Broadcasting state: " << outStr << "\n")
            sendJson(outStr); 
        }

        // Sleep until next tick
        nextTime += period;
        std::this_thread::sleep_until(nextTime);
    }
}

/**********************************************************/
/* handleCommand                                          */
/**********************************************************/
void RealTimeDaemon::handleCommand(const std::string& jsonStr)
{
    IFDEBUG(std::cout << "[RealTimeDaemon][DEBUG] Handling command: " << jsonStr << "\n")
    // parse JSON
    Json::CharReaderBuilder rb;
    Json::Value root;
    std::string errs;
    std::unique_ptr<Json::CharReader> reader(rb.newCharReader());
    bool ok = reader->parse(jsonStr.data(), jsonStr.data() + jsonStr.size(), &root, &errs);
    if (!ok) {
        std::cerr << "[RealTimeDaemon] Invalid JSON: " << errs << "\n";
        return;
    }

    if (!root.isObject()) return;
    std::string cmd = root["cmd"].asString();
    int motorID = root.get("motorID", 1).asInt();
    IFDEBUG(std::cout << "[RealTimeDaemon][DEBUG] Command parsed: " << cmd << " for motorID " << motorID << "\n")

    try {
        auto &mot = m_robot.getMotor(motorID);

        if (cmd == "motorOn") {
            mot.motorOn();
        }
        else if (cmd == "motorOff") {
            mot.motorOff();
        }
        else if (cmd == "motorStop") {
            mot.motorStop();
        }
        else if (cmd == "setTorque") {
            int val = root["value"].asInt();
            mot.setTorque(val);
        }
        else if (cmd == "setPID") {
            uint8_t akp = root["angKp"].asUInt();
            uint8_t aki = root["angKi"].asUInt();
            uint8_t skp = root["spdKp"].asUInt();
            uint8_t ski = root["spdKi"].asUInt();
            uint8_t tkp = root["iqKp"].asUInt();
            uint8_t tki = root["iqKi"].asUInt();
            mot.writePID_RAM(akp, aki, skp, ski, tkp, tki);
        }
        else if (cmd == "setPosition") {
            int32_t position = root["value"].asInt();
            mot.setMultiAngle(position);
        }
        // Additional commands can be added here as needed.
    } catch (std::exception &ex) {
        std::cerr << "[RealTimeDaemon] handleCommand exception: " << ex.what() << "\n";
    }
}

/**********************************************************/
/* sendJson                                               */
/**********************************************************/
void RealTimeDaemon::sendJson(const std::string& jsonStr)
{
    // Append a newline to ensure the Node side can correctly detect message boundaries.
    std::string jsonWithNewline = jsonStr + "\n";
    IFDEBUG(std::cout << "[RealTimeDaemon][DEBUG] sendJson called with: " << jsonWithNewline << "\n")
    
    // Send the JSON string to all connected clients.
    std::lock_guard<std::mutex> lk(m_clientFdsMutex);
    for (auto it = m_clientFds.begin(); it != m_clientFds.end(); ) {
        ssize_t written = write(*it, jsonWithNewline.c_str(), jsonWithNewline.size());
        if (written < 0) {
            std::cerr << "[RealTimeDaemon] Error writing to client FD=" << *it << ". Closing connection.\n";
            close(*it);
            it = m_clientFds.erase(it);
        } else {
            ++it;
        }
    }
}

#ifndef UTILS_HPP
#define UTILS_HPP

// Global Debug Flag
#ifdef DEBUG

    // CAN Debugging
    #ifdef CAN_DEBUG
        #define IFCANDEBUG(x) do { x; } while (0)
    #else
        #define IFCANDEBUG(x) do {} while (0)
    #endif

    // Realtime Debugging
    #ifdef REALTIME_DEBUG
        #define IFRTDEBUG(x) do { x; } while (0)
    #else
        #define IFRTDEBUG(x) do {} while (0)
    #endif

    // General Debugging
    #define IFDEBUG(x) do { x; } while (0)

#else
    #define IFCANDEBUG(x) do {} while (0)
    #define IFRTDEBUG(x) do {} while (0)
    #define IFDEBUG(x) do {} while (0)
#endif

#define LOG_ERROR(x) std::cerr << "[ERROR] " << x << std::endl;
#define LOG_WARNING(x) std::cerr << "[WARNING] " << x << std::endl;
#define LOG_INFO(x) std::cout << "[INFO] " << x << std::endl;

#endif // UTILS_HPP

cmake_minimum_required(VERSION 3.10)
project(armatron_controls)

set(CMAKE_CXX_STANDARD 17)

# Include headers
include_directories(include)

# Core source files shared by both executables
set(CORE_SOURCES
    src/can_handler.cpp
    src/motor_interface.cpp
    src/robot_interface.cpp
    src/real_time_daemon.cpp
)

# ============ Debug Executable ============

add_executable(armatron_debug
    main_debug.cpp
    ${CORE_SOURCES}
)

find_package(Threads REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)

target_link_libraries(armatron_debug Threads::Threads rt ${JSONCPP_LIBRARIES})

# ============ Real-Time Daemon ============

add_executable(realtime_daemon
    main_realtime.cpp
    ${CORE_SOURCES}
)


target_link_libraries(realtime_daemon Threads::Threads rt ${JSONCPP_LIBRARIES})

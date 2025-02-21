cmake_minimum_required(VERSION 3.10)
project(armatron_software)

set(CMAKE_CXX_STANDARD 17)

# Include directories
include_directories(include)

# Source files
set(SOURCES
    src/can_handler.cpp
    src/motor_interface.cpp
    src/robot_interface.cpp
    src/main.cpp
)

# Executable
add_executable(armatron_software ${SOURCES})

# Libraries
find_package(Threads REQUIRED)
target_link_libraries(armatron_software Threads::Threads)
target_link_libraries(armatron_software rt)

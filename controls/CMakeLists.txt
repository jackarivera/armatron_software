cmake_minimum_required(VERSION 3.10)
project(armatron_controls)

set(CMAKE_CXX_STANDARD 17)

# If using RT libraries or POSIX calls:
# find_package(Threads REQUIRED)

# Source files
file(GLOB CONTROLS_SOURCES
    "src/*.cpp"
    "main.cpp"
)

add_executable(armatron_controls ${CONTROLS_SOURCES})
target_include_directories(armatron_controls PRIVATE include)

# Link any libraries you need:
# target_link_libraries(armatron_controls PRIVATE Threads::Threads rt)

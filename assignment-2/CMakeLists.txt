cmake_minimum_required(VERSION 2.8.11)
project(assignment_2)

set(CMAKE_BUILD_TYPE Debug)

include_directories(BEFORE ${PROJECT_SOURCE_DIR}/../include /opt/pumasim/include)

# creates a convenient link to pumasim in the build directory
add_custom_command(
    TARGET fowardkinematics POST_BUILD
    COMMAND ln -sf ../pumasim pumasim
)

# For compilation of  the control loop library:
add_library(fowardkinematics SHARED
    forwardkinematics/forwardkinematics.cpp
    forwardkinematics/forwardkinematics.hpp
)
target_link_libraries (fowardkinematics)
cmake_minimum_required(VERSION 2.8.11)
project(assignment_2)

set(CMAKE_BUILD_TYPE Debug)

include_directories(BEFORE ${PROJECT_SOURCE_DIR}/../include /opt/pumasim/include)

# For compilation of the forwardkinematics library
add_library(forwardkinematics SHARED
	forwardkinematics/forwardkinematics.cpp
	forwardkinematics/forwardkinematics.hpp
)

# For compilation of the controlDLL library
add_library(controlDLL SHARED
	forwardkinematics/forwardkinematics.cpp
	forwardkinematics/forwardkinematics.hpp
	controlDLL/control.cpp
)

# Add a custom target to create the symbolic link
add_custom_target(create_pumasim_symlink ALL
	COMMAND ln -sf ../pumasim pumasim
	COMMENT "Creating symbolic link to pumasim"
)

# Make the libraries depend on the custom target
add_dependencies(forwardkinematics create_pumasim_symlink)
add_dependencies(controlDLL create_pumasim_symlink)

# Link controlDLL with forwardkinematics
target_link_libraries(controlDLL forwardkinematics)
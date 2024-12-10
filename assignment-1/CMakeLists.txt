cmake_minimum_required(VERSION 2.8.11)
project(assignment_1)

set(CMAKE_BUILD_TYPE Debug)

include_directories(BEFORE ${PROJECT_SOURCE_DIR}/../include /opt/pumasim/include)

add_library(controlDLL SHARED
	controlDLL/control.cpp
)

# Add a custom target to create the symbolic link
add_custom_target(create_pumasim_symlink ALL
	COMMAND ln -sf ../pumasim pumasim
	COMMENT "Creating symbolic link to pumasim"
)

# Make the library depend on the custom target
add_dependencies(controlDLL create_pumasim_symlink)

target_link_libraries(controlDLL)
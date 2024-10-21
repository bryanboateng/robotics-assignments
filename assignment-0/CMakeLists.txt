cmake_minimum_required(VERSION 3.20)
project(Assignment0)

set(CMAKE_CXX_STANDARD 11)

set(MAIN_SOURCE_FILES
        SpringMass.cpp
        SpringDamperMass.cpp
        main.cpp
)
set(EXPORT_SOURCE_FILES
        SpringMass.cpp
        SpringDamperMass.cpp
        export.cpp
)
# Add the executable
add_executable(main ${MAIN_SOURCE_FILES})
add_executable(export ${EXPORT_SOURCE_FILES})
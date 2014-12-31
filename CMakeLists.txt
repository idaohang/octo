cmake_minimum_required(VERSION 2.8)

# Project definition
project(Octo)
enable_language(CXX)

# C++ Config
set(CMAKE_CXX_FLAGS "-std=c++0x -O3 -Wall")

# Append custom cmake modules for this project to the CMAKE_MODULE_PATH
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)

# Output executable files to a bin/ directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)

# Allow us to include headers
include_directories(${PROJECT_SOURCE_DIR}/include)

# Compile the code
add_subdirectory(src)


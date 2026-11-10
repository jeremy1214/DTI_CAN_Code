cmake_minimum_required(VERSION 3.14)
project(DTI_CAN_Code VERSION 2.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include directories
include_directories(include)

# Core library
add_library(dti_can_core STATIC
    src/dti_codec.cpp
    src/dti_receiver.cpp
    src/dti_logger.cpp
)

target_include_directories(dti_can_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Shared library (optional DLL for external programs / languages)
add_library(dti_can_shared SHARED
    src/dti_codec.cpp
    src/dti_receiver.cpp
    src/dti_logger.cpp
)
target_include_directories(dti_can_shared PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# Test executable
add_executable(dti_test examples/test_codec.cpp)
target_link_libraries(dti_test PRIVATE dti_can_core)

# Demo executable
add_executable(dti_demo examples/demo_integration.cpp)
target_link_libraries(dti_demo PRIVATE dti_can_core)

cmake_minimum_required(VERSION 3.10)
project(cpp_sdk VERSION 1.0.0)

# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -O2")

# OpenCV
find_package(OpenCV REQUIRED)
if(OpenCV_FOUND)
    message(STATUS "OpenCV found: ${OpenCV_VERSION}")
    message(STATUS "OpenCV include dirs: ${OpenCV_INCLUDE_DIRS}")
    message(STATUS "OpenCV libraries: ${OpenCV_LIBS}")
endif()

# Threads
find_package(Threads REQUIRED)

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
)

# SDK library sources
set(SDK_SOURCES
    src/das_protocol.cpp
    src/pack.cpp
    src/databus.cpp
    src/camera.cpp
    src/gripper_system.cpp
    src/tactile_processing.cpp
)

# Static library
add_library(cpp_sdk_static STATIC ${SDK_SOURCES})
target_link_libraries(cpp_sdk_static
    ${OpenCV_LIBS}
    Threads::Threads
)

# Shared library
add_library(cpp_sdk SHARED ${SDK_SOURCES})
target_link_libraries(cpp_sdk
    ${OpenCV_LIBS}
    Threads::Threads
)

# start_gripper: run root start_gripper.cpp directly (self-compiles); not built here

# Camera calibration CLI: camera_cmd
add_executable(camera_cmd src/camera_cmd.cpp)
target_link_libraries(camera_cmd
    cpp_sdk_static
    ${OpenCV_LIBS}
    Threads::Threads
)

# Install rules
install(TARGETS camera_cmd cpp_sdk cpp_sdk_static
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

install(DIRECTORY include/ DESTINATION include)

# Ensure calib_result exists
file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/calib_result)

# Build summary
message(STATUS "")
message(STATUS "=== cpp_sdk Build Configuration ===")
message(STATUS "")
message(STATUS "Project: ${PROJECT_NAME}")
message(STATUS "Version: ${PROJECT_VERSION}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "")
message(STATUS "OpenCV Version: ${OpenCV_VERSION}")
message(STATUS "")
message(STATUS "Targets:")
message(STATUS "  - start_gripper.cpp (run directly: ./start_gripper.cpp <args>)")
message(STATUS "  - camera_cmd (executable)")
message(STATUS "  - cpp_sdk (shared library)")
message(STATUS "  - cpp_sdk_static (static library)")
message(STATUS "")
message(STATUS "Build commands:")
message(STATUS "  mkdir -p build && cd build && cmake .. && make")
message(STATUS "")

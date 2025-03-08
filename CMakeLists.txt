cmake_minimum_required(VERSION 3.5)
project(RealSenseOpenCVExample)

# Find packages
find_package(realsense2 REQUIRED)
find_package(OpenCV REQUIRED)

# Set C++ standard
set(CMAKE_CXX_STANDARD 11)

# Add executable target
# add_executable(main src/test.cpp)
# add_executable(main src/with_clip.cpp)
# add_executable(main src/better.cpp)
add_executable(main src/version2.cpp)
add_executable(get_max_dis src/get_max_dis.cpp
        src/version_3.cpp)

# Link libraries
target_link_libraries(main PRIVATE realsense2::realsense2 ${OpenCV_LIBS})
target_link_libraries(get_max_dis PRIVATE realsense2::realsense2 ${OpenCV_LIBS})

# Set include directories
target_include_directories(main PRIVATE ${OpenCV_INCLUDE_DIRS})
target_include_directories(get_max_dis PRIVATE ${OpenCV_INCLUDE_DIRS})

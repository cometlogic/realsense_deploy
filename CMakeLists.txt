cmake_minimum_required(VERSION 3.5)
project(RealSenseOpenCVExample)

# Find packages
find_package(realsense2 REQUIRED)
find_package(OpenCV REQUIRED)

# Set C++ standard
set(CMAKE_CXX_STANDARD 11)

# Add executable target
# add_executable(main src/colormap.cpp)
# add_executable(main src/with_clip.cpp)
# add_executable(main src/better.cpp)
add_executable(version_2 src/version2.cpp)
add_executable(get_max_dis src/get_max_dis.cpp)
add_executable(version_3 src/version3.cpp)
add_executable(version_4 src/version4.cpp)
add_executable(align src/align.cpp)
add_executable(align_inpaint src/align_inpaint.cpp)

add_executable(test test/speed_test.cpp)
target_link_libraries(test PRIVATE realsense2::realsense2 ${OpenCV_LIBS})
target_include_directories(test PRIVATE  ${OpenCV_INCLUDE_DIRS})

# Link libraries
target_link_libraries(version_2 PRIVATE realsense2::realsense2 ${OpenCV_LIBS})
target_link_libraries(version_3 PRIVATE realsense2::realsense2 ${OpenCV_LIBS})
target_link_libraries(version_4 PRIVATE realsense2::realsense2 ${OpenCV_LIBS})
target_link_libraries(get_max_dis PRIVATE realsense2::realsense2 ${OpenCV_LIBS})
target_link_libraries(align PRIVATE realsense2::realsense2 ${OpenCV_LIBS})
target_link_libraries(align_inpaint PRIVATE realsense2::realsense2 ${OpenCV_LIBS})

# Set include directories
target_include_directories(version_2 PRIVATE ${OpenCV_INCLUDE_DIRS})
target_include_directories(version_3 PRIVATE ${OpenCV_INCLUDE_DIRS})
target_include_directories(version_4 PRIVATE ${OpenCV_INCLUDE_DIRS})
target_include_directories(get_max_dis PRIVATE ${OpenCV_INCLUDE_DIRS})
target_include_directories(align PRIVATE ${OpenCV_INCLUDE_DIRS})
target_include_directories(align_inpaint PRIVATE ${OpenCV_INCLUDE_DIRS})

cmake_minimum_required(VERSION 3.10)
project(inverted_pendulum)

set(Boost_USE_STATIC_LIBS OFF) 
set(Boost_USE_MULTITHREADED ON)  
set(Boost_USE_STATIC_RUNTIME OFF) 
find_package(Boost REQUIRED) 

# set(OpenCV_DIR "/home/ehambram/workspaces/libraries/OpenCV/opencv-4.5.1/build") // build on HP
# find_package(OpenCV 4.5.1 REQUIRED)
set(OpenCV_DIR "/home/ehambram/workspaces/libraries/opencv/opencv-3.4.13/build")
find_package(OpenCV REQUIRED)

find_package (Threads REQUIRED)

set(CMAKE_CXX_STANDARD 17)

include_directories(${Boost_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} include) 
add_executable(inverted_pendulum_model src/main.cpp src/InvertedPendulum.cpp) 
target_link_libraries(inverted_pendulum_model ${Boost_LIBRARIES} ${OpenCV_LIBS} ${CMAKE_THREAD_LIBS_INIT})
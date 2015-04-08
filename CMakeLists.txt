cmake_minimum_required(VERSION 2.8.3)
project(flea3)

find_package(catkin REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)

find_package(catkin REQUIRED COMPONENTS
    roscpp nodelet camera_base message_generation
    )

generate_dynamic_reconfigure_options(cfg/Flea3Dyn.cfg)

catkin_package()

# We don't include FlyCapture2 because it is installed to /usr/include
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    )



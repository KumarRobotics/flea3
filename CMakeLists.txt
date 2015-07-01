cmake_minimum_required(VERSION 2.8.3)
project(flea3)

find_package(catkin REQUIRED)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")

find_package(catkin REQUIRED COMPONENTS
    roscpp nodelet camera_base message_generation
    )

find_library(FlyCapture2_LIBRARIES flycapture)
if(FlyCapture2_LIBRARIES)
    message(STATUS "FlyCapture2 libraries found: " ${FlyCapture2_LIBRARIES})
else()
    message(STATUS "FlyCapture2 libraries not found")
endif()

generate_dynamic_reconfigure_options(cfg/Flea3Dyn.cfg)

catkin_package()

# We don't include FlyCapture2 because it is installed to /usr/include
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    )

add_library(${PROJECT_NAME}
    src/flea3_camera.cpp
    src/flea3_ros.cpp
    src/single/single_node.cpp
    )
target_link_libraries(
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${FlyCapture2_LIBRARIES}
    )

add_executable(flea3_single_main src/single/single_main.cpp)
target_link_libraries(flea3_single_main ${PROJECT_NAME})

add_executable(flea3_list_cameras src/list_cameras.cpp)
target_link_libraries(flea3_list_cameras ${PROJECT_NAME})

add_dependencies(${PROJECT_NAME}
    ${catkin_EXPORTED_TARGETS}
    ${PROJECT_NAME}_gencfg
    ${PROJECT_NAME}_gencpp
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    )

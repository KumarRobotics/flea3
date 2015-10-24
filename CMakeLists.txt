cmake_minimum_required(VERSION 2.8.3)
project(flea3)

find_package(catkin REQUIRED)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")

find_package(catkin REQUIRED COMPONENTS
    roscpp nodelet camera_base message_generation std_msgs
    )

find_library(FlyCapture2_LIBRARIES flycapture)
if(FlyCapture2_LIBRARIES)
    message(STATUS "FlyCapture2 libraries found: " ${FlyCapture2_LIBRARIES})
else()
    message(STATUS "FlyCapture2 libraries not found")
endif()

# Dynamic reconfigure
generate_dynamic_reconfigure_options(cfg/Flea3Dyn.cfg)

# Message
add_message_files(DIRECTORY msg
    FILES ImageMetadata.msg
    )
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
    CATKIN_DEPENDS message_runtime std_msgs
    )

# We don't include FlyCapture2 because it is installed to /usr/include
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    )

add_library(${PROJECT_NAME}
    src/flea3_camera.cpp
    src/flea3_setting.cpp
    src/flea3_ros.cpp
    src/single/single_node.cpp
    src/stereo/stereo_node.cpp
    src/single/single_nodelet.cpp
    )
target_link_libraries(
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${FlyCapture2_LIBRARIES}
    )

add_executable(flea3_single_node src/single/single_main.cpp)
target_link_libraries(flea3_single_node ${PROJECT_NAME})

add_executable(flea3_stereo_node src/stereo/stereo_main.cpp)
target_link_libraries(flea3_stereo_node ${PROJECT_NAME})

add_executable(flea3_list_cameras src/list_cameras.cpp)
target_link_libraries(flea3_list_cameras ${PROJECT_NAME})

add_executable(async_trigger src/flycapture/async_trigger.cpp)
target_link_libraries(async_trigger ${FlyCapture2_LIBRARIES})

add_dependencies(${PROJECT_NAME}
    ${catkin_EXPORTED_TARGETS}
    ${PROJECT_NAME}_gencfg
    ${PROJECT_NAME}_gencpp
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    )

install(TARGETS flea3_single_node flea3_stereo_node flea3_list_cameras async_trigger
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN "*.*~" EXCLUDE
)

install(DIRECTORY msg/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/msg
  PATTERN "*.*~" EXCLUDE
)

install(DIRECTORY cfg/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cfg
  PATTERN "*.*~" EXCLUDE
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN "*.*~" EXCLUDE
)

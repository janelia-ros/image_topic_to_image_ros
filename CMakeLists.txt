cmake_minimum_required(VERSION 2.8.3)
project(image_stream_to_heatmap)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  cv_bridge
  image_transport
  camera_info_manager
  sensor_msgs
  nodelet
  roslaunch
)

roslaunch_add_file_check(launch)

find_package(OpenCV REQUIRED)
if (OpenCV_VERSION VERSION_EQUAL "3")
  add_definitions("-DOPENCV3=1")
endif()

find_package(Boost REQUIRED COMPONENTS thread)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs
  DEPENDS OpenCV
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
include_directories(include)

add_library(${PROJECT_NAME}
  src/nodelets/create_file.cpp
  src/libimage_stream_to_heatmap/advertisement_checker.cpp
)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBRARIES})

# Standalone node
add_executable(image_stream_to_heatmap_exe src/nodes/image_stream_to_heatmap.cpp)
target_link_libraries(image_stream_to_heatmap_exe ${PROJECT_NAME}  ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBRARIES})
SET_TARGET_PROPERTIES(image_stream_to_heatmap_exe PROPERTIES OUTPUT_NAME image_stream_to_heatmap)
install(TARGETS image_stream_to_heatmap_exe
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

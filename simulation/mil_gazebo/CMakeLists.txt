cmake_minimum_required(VERSION 2.8.3)
project(mil_gazebo)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  gazebo_dev
  gazebo_ros
  gazebo_plugins
  mil_msgs
  image_geometry
  point_cloud_object_detection_and_recognition
)

catkin_package(INCLUDE_DIRS include)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    include
)

# Shared Utilities
add_library(
    mil_gazebo_utils
        src/mil_gazebo_utils.cpp
)
target_link_libraries(mil_gazebo_utils
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)
add_dependencies(mil_gazebo_utils ${catkin_EXPORTED_TARGETS})


add_library(
    mil_model_heading
        src/mil_model_heading.cpp
)
target_link_libraries(mil_model_heading
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)
add_dependencies(mil_model_heading ${catkin_EXPORTED_TARGETS})

# Add buoyancy plugin
add_library(
    mil_buoyancy_gazebo
        src/mil_buoyancy_gazebo.cpp
)
target_link_libraries(mil_buoyancy_gazebo
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  mil_gazebo_utils
)
add_dependencies(mil_buoyancy_gazebo mil_gazebo_utils ${catkin_EXPORTED_TARGETS})

# Add DVL plugin
add_library(
    mil_dvl_gazebo
        src/mil_dvl_gazebo.cpp
)
target_link_libraries(mil_dvl_gazebo
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  mil_gazebo_utils
)
add_dependencies(mil_dvl_gazebo mil_gazebo_utils ${catkin_EXPORTED_TARGETS})

# Add Magnetometer plugin
add_library(
    mil_magnetometer_gazebo
        src/mil_magnetometer_gazebo.cpp
)
target_link_libraries(mil_magnetometer_gazebo
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  mil_gazebo_utils
)
add_dependencies(mil_magnetometer_gazebo mil_gazebo_utils ${catkin_EXPORTED_TARGETS})

# Add Depth plugin
add_library(
    mil_depth_gazebo
        src/mil_depth_gazebo.cpp
)
target_link_libraries(mil_depth_gazebo
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  mil_gazebo_utils
)
add_dependencies(mil_depth_gazebo mil_gazebo_utils ${catkin_EXPORTED_TARGETS})

# Add IMU Plugin
add_library(
    mil_imu_gazebo
        src/mil_imu_gazebo.cpp
)
target_link_libraries(mil_imu_gazebo
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  mil_gazebo_utils
)
add_dependencies(mil_imu_gazebo mil_gazebo_utils ${catkin_EXPORTED_TARGETS})

# Add Drag plugin
add_library(
    mil_drag_gazebo
        src/mil_drag_gazebo.cpp
)
target_link_libraries(mil_drag_gazebo
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  mil_gazebo_utils
)
add_dependencies(mil_drag_gazebo mil_gazebo_utils ${catkin_EXPORTED_TARGETS})


# Add pcodar_gazebo library
add_library(
    pcodar_gazebo
        src/pcodar_gazebo.cpp
)
target_link_libraries(pcodar_gazebo
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)
add_dependencies(pcodar_gazebo
  ${catkin_EXPORTED_TARGETS}
)


#Add image_ground_truth plugin
add_library(
    mil_image_ground_truth
        src/mil_image_ground_truth.cpp
)
target_link_libraries(mil_image_ground_truth
  ${GAZEBO_PLUGINS} 
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)
add_dependencies(mil_image_ground_truth
  ${catkin_EXPORTED_TARGETS}
  mil_msgs
  image_geometry
)



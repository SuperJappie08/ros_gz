cmake_minimum_required(VERSION 3.5)

project(ros_gz_point_cloud)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# find_package(ignition-gazebo2 2.1 QUIET REQUIRED)
# set(GZ_SIM_VER ${ignition-gazebo2_VERSION_MAJOR})
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)

find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED)

# find_package(ignition-rendering2 QUIET REQUIRED)
find_package(gz_rendering_vendor REQUIRED)
find_package(gz-rendering REQUIRED)
# set(GZ_RENDERING_VER ${ignition-rendering2_VERSION_MAJOR})

# find_package(ignition-sensors2 QUIET REQUIRED)
# set(GZ_SENSORS_VER ${ignition-sensors2_VERSION_MAJOR})
find_package(gz_sensors_vendor REQUIRED)
find_package(gz-sensors REQUIRED)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

include_directories(include)

set(plugin_name ros-gz-point-cloud-system)
add_library(${plugin_name} SHARED
  src/point_cloud.cc
)
target_link_libraries(${plugin_name}
  gz-sim::gz-sim
  gz-rendering::core
  gz-sensors::core
  gz-plugin::register
)

ament_target_dependencies(${plugin_name}
  "rclcpp"
  "sensor_msgs"
)

# install(DIRECTORY
#   include/
#   DESTINATION include
# )

install(TARGETS ${plugin_name}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY
    examples/
  DESTINATION
    examples
)


# ament_export_include_directories(include)
ament_export_libraries(${plugin_name})

# # TODO(CH3): Install symlink to deprecated library name. Remove on tock.
# string(REPLACE "RosGz" "RosIgn" plugin_name_ign ${plugin_name})
# if(WIN32)
  # symlinks on Windows require admin priviledges, fallback to copy
  # ADD_CUSTOM_COMMAND(TARGET ${plugin_name} POST_BUILD
    # COMMAND "${CMAKE_COMMAND}" -E copy
      # "$<TARGET_FILE:${plugin_name}>"
      # "$<TARGET_FILE_DIR:${CATKIN_PACKAGE_LIB_DESTINATION}>/${plugin_name_ign}")
# else()
#   file(MAKE_DIRECTORY "${PROJECT_BINARY_DIR}/lib")
  # EXECUTE_PROCESS(COMMAND ${CMAKE_COMMAND} -E create_symlink ${plugin_name} ${plugin_name_ign} WORKING_DIRECTORY "${PROJECT_BINARY_DIR}/lib")
  # INSTALL(FILES ${PROJECT_BINARY_DIR}/lib/${plugin_name_ign} DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
# endif()

ament_package()

# SPDX-FileCopyrightText: 2025, FANUC America Corporation
# SPDX-FileCopyrightText: 2025, FANUC CORPORATION
#
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.5)
project(fanuc_moveit_config)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(moveit_ros_control_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(moveit_core REQUIRED)
# find_package(ruckig REQUIRED)
find_package(srdfdom REQUIRED)

include(GenerateExportHeader)

add_library(fanuc_controllers_moveit_trajectory_plugin SHARED
    src/fanuc_controllers_moveit_trajectory_plugin.cpp
)
target_link_libraries(fanuc_controllers_moveit_trajectory_plugin PRIVATE
    moveit_ros_control_interface::moveit_ros_control_interface_trajectory_plugin)

pluginlib_export_plugin_description_file(moveit_ros_control_interface fanuc_controllers_moveit_trajectory_plugin.xml)

# ruckig
# add_library(moveit_ruckig_filter SHARED src/ruckig_filter.cpp)
# generate_export_header(moveit_ruckig_filter)
# target_include_directories(
#   moveit_ruckig_filter PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
# generate_parameter_library(moveit_ruckig_filter_parameters
#                            src/ruckig_filter_parameters.yaml)
# target_link_libraries(
#   moveit_ruckig_filter
#   moveit_core::moveit_robot_state
#   moveit_ruckig_filter_parameters
#   moveit_core::moveit_smoothing_base
#   ruckig::ruckig
#   srdfdom::srdfdom
#   pluginlib::pluginlib)

# pluginlib_export_plugin_description_file(moveit_core filter_plugin_ruckig.xml)

install(FILES
    fanuc_controllers_moveit_trajectory_plugin.xml
    DESTINATION share/${PROJECT_NAME}
)
install(TARGETS
    fanuc_controllers_moveit_trajectory_plugin
    DESTINATION lib/${PROJECT_NAME}
)

# install(FILES ${CMAKE_CURRENT_BINARY_DIR}/moveit_ruckig_filter_export.h
#         DESTINATION include/moveit_core)
# install(TARGETS
#     moveit_ruckig_filter
#     DESTINATION lib/${PROJECT_NAME}
# )

install(DIRECTORY config launch rviz srdf
    DESTINATION share/${PROJECT_NAME}
)

ament_package()

cmake_minimum_required(VERSION 3.5.0)
project(mutable_transform_publisher)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mutable_transform_publisher_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(std_srvs REQUIRED)

###########
## Build ##
###########

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
)


# Core ROS capability
add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/publisher.cpp
  src/${PROJECT_NAME}/mutable_transform_publisher.cpp
)

ament_target_dependencies(${PROJECT_NAME}
                          "geometry_msgs"
                          "rclcpp"
                          "tf2"
                          "tf2_ros"
                          "mutable_transform_publisher_msgs"
			  "std_srvs"
                          )

target_link_libraries(${PROJECT_NAME}
                      ${rclcpp_LIBRARIES}
                      ${tf2_LIBRARIES}
                      ${tf2_ros_LIBRARIES}
                      ${geometry_msgs_LIBRARIES}
                      )

# Yaml Stuff
add_library(${PROJECT_NAME}_yaml
  src/${PROJECT_NAME}/yaml_serialization.cpp
)

ament_target_dependencies(${PROJECT_NAME}_yaml ${${PROJECT_NAME}_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_yaml
                      ${YAML_CPP_LIBRARIES}
                      )

# Primary Node
add_executable(${PROJECT_NAME}_node src/mutable_transform_publisher_node.cpp)

set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME mutable_transform_publisher PREFIX "")

ament_target_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} ${PROJECT_NAME}_yaml)

ament_export_include_directories(include
                                ${YAML_CPP_INCLUDE_DIRS}
                                )

ament_export_libraries(${PROJECT_NAME}
                       ${PROJECT_NAME}_yaml
                       ${PROJECT_NAME}_node
                       )

ament_export_dependencies(rclcpp
                          )

if (BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/launch_testing/mutable_transform_publisher_launch_test.py)
endif()

ament_package()

#############
## Install ##
#############

set(ROS_LIB_DESTINATION lib)
set(ROS_BIN_DESTINATION bin)
set(ROS_INCLUDE_DESTINATION include)

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_yaml ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${ROS_LIB_DESTINATION}
  LIBRARY DESTINATION ${ROS_LIB_DESTINATION}
  RUNTIME DESTINATION ${ROS_BIN_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}_node
        DESTINATION lib/${PROJECT_NAME})

# Install the library
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(DIRECTORY test
        DESTINATION share/${PROJECT_NAME})

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
 DESTINATION ${ROS_INCLUDE_DESTINATION}
 FILES_MATCHING PATTERN "*.h"
)

cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME as2_control_command_handlers)
project(${PROJECT_NAME})

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(aerostack2_core REQUIRED)
find_package(aerostack2_msgs REQUIRED)
find_package(std_srvs REQUIRED)


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
include_directories(
  include
  include/${PROJECT_NAME}
)

set(SOURCE_CPP_FILES 
  src/basic_control_commands.cpp
  src/position_control.cpp
  src/speed_control.cpp
  )

set(THIS_PROJECT_DEPENDENCIES
rclcpp
rclcpp_action
std_msgs
sensor_msgs
geometry_msgs
nav_msgs
aerostack2_core
aerostack2_msgs
std_srvs
)

add_library(${PROJECT_NAME} ${SOURCE_CPP_FILES})
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME} ${THIS_PROJECT_DEPENDENCIES})
ament_export_include_directories(include)    
ament_export_interfaces(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  ament_cmake
  ${THIS_PROJECT_DEPENDENCIES}
)
ament_export_libraries(${PROJECT_NAME})




if(BUILD_TESTING)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_clang_format REQUIRED)
  
  ament_cpplint(src/ include/)
  ament_cppcheck(src/ include/)
  ament_clang_format(src/ include/)

endif()

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()

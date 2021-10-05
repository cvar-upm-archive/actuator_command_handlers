cmake_minimum_required(VERSION 3.5)
project(pixhawk_test)
set(PROJECT_NAME as2_control_command_handlers)

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
set(SOURCE_CPP_FILES 
  src/${PROJECT_NAME}/basic_control_commands.cpp
  src/${PROJECT_NAME}/position_control.cpp
  src/${PROJECT_NAME}/speed_control.cpp
  )

set(HEADER_HPP_FILES
  include/${PROJECT_NAME}/basic_control_commands.hpp
  include/${PROJECT_NAME}/position_control.hpp
  include/${PROJECT_NAME}/speed_control.hpp
)

include_directories(
  include
  include/${PROJECT_NAME}
  ${aerostack2_core_INCLUDE_DIRS}
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

add_library(${PROJECT_NAME} SHARED ${SOURCE_CPP_FILES})
ament_target_dependencies(${PROJECT_NAME} ${THIS_PROJECT_DEPENDENCIES})
ament_export_interfaces(${PROJECT_NAME}_export HAS_LIBRARY_TARGET)
ament_export_libraries(${PROJECT_NAME})
ament_export_include_directories(include)
ament_export_dependencies(
  ament_cmake
  ${THIS_PROJECT_DEPENDENCIES}
)


if(BUILD_TESTING)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_clang_format REQUIRED)
  
  ament_cpplint(src/ include/)
  ament_cppcheck(src/ include/)
  ament_clang_format(src/ include/)

endif()

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}_export
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin/${PROJECT_NAME}
)
    
install(
  DIRECTORY include/
  DESTINATION include
)


ament_package()

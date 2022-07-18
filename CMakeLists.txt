cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME actuator_command_handlers)
project(${PROJECT_NAME} VERSION 0.2.0)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# set Release as default
if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

#set fPIC to ON by default
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(PROJECT_DEPENDENCIES
  ament_cmake
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  as2_core
  as2_msgs
  std_srvs
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
include_directories(
  include
  include/${PROJECT_NAME}
)

set(SOURCE_CPP_FILES 
  src/basic_actuator_commands.cpp
  src/position_control.cpp
  src/speed_control.cpp
  src/acro_control.cpp
  )


add_library(${PROJECT_NAME} ${SOURCE_CPP_FILES})
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME} ${PROJECT_DEPENDENCIES})
ament_export_include_directories(include)    
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  ament_cmake
  ${PROJECT_DEPENDENCIES}
)
ament_export_libraries(${PROJECT_NAME})



if(BUILD_TESTING)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_clang_format REQUIRED)
  
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

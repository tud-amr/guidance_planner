cmake_minimum_required(VERSION 3.5)
project(guidance_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# OPENMP (for parallel computation)
find_package(OpenMP)
if(OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
else()
  message(WARNING "Compiling without OPENMP (OPENMP not found)")
endif()

# General requirements
set(DEPENDENCIES
  rclcpp
  std_msgs 
  ros_tools
)

# GSL (for integration)
find_package(PkgConfig REQUIRED)
pkg_check_modules(gsl REQUIRED gsl)

find_package(ament_cmake REQUIRED)
foreach(pkg IN LISTS DEPENDENCIES)
  find_package(${pkg} REQUIRED)
endforeach()

include_directories(
  include
)

add_library(${PROJECT_NAME}_core SHARED
  src/config.cpp
  src/paths.cpp
  src/types.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
ament_target_dependencies(${PROJECT_NAME}_core ros_tools)
target_link_libraries(${PROJECT_NAME}_core ${gsl_LIBRARIES})

# Library
add_library(${PROJECT_NAME} SHARED
  src/global_guidance.cpp
  src/prm.cpp
  src/graph.cpp
  src/environment.cpp
  src/graph_search.cpp
  src/homology.cpp
  src/uvd.cpp
  src/winding_angle.cpp
  src/cubic_spline.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

ament_target_dependencies(${PROJECT_NAME} ${DEPENDENCIES})
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_core ${gsl_LIBRARIES})

# Example
add_executable(example src/ros2_example.cpp)
target_link_libraries(example ${PROJECT_NAME})

## INSTALL
install(
  TARGETS 
  ${PROJECT_NAME}_core
  ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS example
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})
  
install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME})

## FINALIZE
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${DEPENDENCIES})
ament_package()
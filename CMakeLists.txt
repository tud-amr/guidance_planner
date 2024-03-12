cmake_minimum_required(VERSION 3.5)
project(guidance_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
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

# find_package(rosidl_default_generators REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ros_tools REQUIRED)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
add_definitions(${EIGEN_DEFINITIONS})

# GSL (for integration)
find_package(PkgConfig REQUIRED)
pkg_check_modules(gsl REQUIRED gsl)

include_directories(
  include
  debug
  ${EIGEN3_INCLUDE_DIRS}
)

# Library
add_library(${PROJECT_NAME} SHARED
  src/global_guidance.cpp
  src/prm.cpp
  src/graph.cpp
  src/paths.cpp
  src/types.cpp
  src/environment.cpp
  src/graph_search.cpp
  src/config.cpp
  src/homology.cpp
  src/cubic_spline.cpp
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  ros_tools
  Eigen3
)
target_link_libraries(${PROJECT_NAME}
  ${gsl_LIBRARIES}
)

# Example
add_executable(example src/ros2_example.cpp)
target_link_libraries(example ${PROJECT_NAME})

## INSTALL
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS example #guidance_server
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
ament_export_dependencies(rclcpp)
ament_export_dependencies(std_msgs)
ament_export_dependencies(ros_tools)
ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(Eigen3)

ament_package()
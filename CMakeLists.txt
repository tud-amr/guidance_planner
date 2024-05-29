cmake_minimum_required(VERSION 3.8)
project(guidance_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(OpenMP)

if(OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
else()
  message(WARNING "Compiling without OPENMP")
endif()

set(DEPENDENCIES
  roscpp
  std_msgs
  ros_tools
  dynamic_reconfigure
)

find_package(catkin REQUIRED COMPONENTS
  ${DEPENDENCIES}
)

find_package(Eigen3 REQUIRED)
add_definitions(${EIGEN_DEFINITIONS})

# ADD GSL FOR HOMOTOPY
find_package(PkgConfig REQUIRED)
pkg_check_modules(gsl REQUIRED gsl)

generate_dynamic_reconfigure_options(cfg/reconfigure.cfg)

catkin_package(
  CATKIN_DEPENDS ${DEPENDENCIES}
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME} ${PROJECT_NAME}_homotopy ${gsl_LIBRARIES}
  DEPENDS EIGEN3
)

include_directories(
  include
  debug
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_homotopy SHARED
  src/homotopy_comparison/uvd.cpp
  src/homotopy_comparison/winding_angle.cpp
  src/homotopy_comparison/homology.cpp
)
add_dependencies(${PROJECT_NAME}_homotopy ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_homotopy
  ${catkin_LIBRARIES}
  ${gsl_LIBRARIES}
)

# Library
add_library(${PROJECT_NAME} SHARED
  src/config.cpp
  src/global_guidance.cpp
  src/prm.cpp
  src/sampler.cpp
  src/graph.cpp
  src/graph_search.cpp
  src/types/paths.cpp
  src/types/types.cpp
  src/types/node.cpp
  src/environment.cpp
  src/cubic_spline.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_homotopy
)

# Example
add_executable(example
  src/ros1_example.cpp
)
add_dependencies(example ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example
  ${PROJECT_NAME}_homotopy
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
add_definitions(-DMPC_PLANNER_ROS)

install(TARGETS
  ${PROJECT_NAME}_homotopy
  ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
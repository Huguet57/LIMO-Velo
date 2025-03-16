cmake_minimum_required(VERSION 3.5)
project(limovelo)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wextra -Wpedantic -O2)
endif()



message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)" )
  include(ProcessorCount)
  ProcessorCount(N)
  message("Processer number:  ${N}")
  if(N GREATER 4)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=3)
    message("core for MP: 3")
  elseif(N GREATER 3)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=2)
    message("core for MP: 2")
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(common_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  include
)

# generate_messages(
#  DEPENDENCIES
#  geometry_msgs
# )

add_executable(limovelo
  include/Headers

  include/ikd-Tree/ikd_Tree/ikd_Tree.h
  include/ikd-Tree/ikd_Tree/ikd_Tree.cpp
  
  include/IKFoM/IKFoM_toolkit
  include/IKFoM/use-ikfom.hpp
  include/IKFoM/use-ikfom.cpp

  # Pipelines
  src/main.cpp
  
  # Utils
  src/Utils/Utils.cpp
  src/Utils/PointCloudProcessor.cpp

  # Objects
  src/Objects/Buffer.cpp
  src/Objects/IMU.cpp
  src/Objects/Match.cpp
  src/Objects/Normal.cpp
  src/Objects/Plane.cpp
  src/Objects/Point.cpp
  src/Objects/RotTransl.cpp
  src/Objects/State.cpp
  
  # Modules
  src/Modules/Accumulator.cpp
  src/Modules/Compensator.cpp
  src/Modules/Localizator.cpp
  src/Modules/Mapper.cpp
)
ament_target_dependencies(
  limovelo 
  geometry_msgs
  nav_msgs
  sensor_msgs
  common_msgs
  rclcpp
  std_msgs
  tf2_sensor_msgs
  tf2_geometry_msgs
  pcl_conversions
  tf2
  tf2_ros
  Eigen3
  PCL
)

target_link_libraries(limovelo ${PCL_LIBRARIES} ${PYTHON_LIBRARIES})
target_include_directories(limovelo
  PUBLIC include/IKFoM/ include/IKFoM/IKFoM_toolkit include/ikd-Tree/ikd_Tree
  PRIVATE ${PYTHON_INCLUDE_DIRS}
)


install(
  TARGETS
  limovelo
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(
  DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  set(ament_cmake_copyright_FOUND TRUE)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

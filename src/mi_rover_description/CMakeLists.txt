cmake_minimum_required(VERSION 3.8)
project(mi_rover_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Gazebo Harmonic (en Jazzy vienen como paquetes vendor; en Humble se ignoran)
find_package(ignition-plugin1 REQUIRED COMPONENTS register)
find_package(ignition-gazebo6 REQUIRED)

# Plugin que fija la postura inicial del brazo una sola vez
add_library(mi_rover_initial_pose SHARED src/initial_joint_positions.cpp)
target_compile_features(mi_rover_initial_pose PRIVATE cxx_std_17)
target_link_libraries(mi_rover_initial_pose
  ignition-plugin1::register
  ignition-gazebo6::ignition-gazebo6)

install(TARGETS mi_rover_initial_pose
  LIBRARY DESTINATION lib)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY urdf meshes launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

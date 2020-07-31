cmake_minimum_required(VERSION 2.8.3)
project(duckietown)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  xacro
)

catkin_python_setup()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# find xacro files
file(GLOB xacro_files urdf/*.urdf.xacro)
xacro_add_files(${xacro_files} INORDER TARGET xacro_files_target)

# add an abstract target to trigger the xacro builds
add_custom_target(xacro_files_target ALL DEPENDS ${xacro_files})

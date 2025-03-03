cmake_minimum_required(VERSION 3.8)
project(group16)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# include folder whish has all the header files
include_directories(include/lecture12)

# Create a variable for all dependencies
set(FRAME_DEMO_INCLUDE_DEPENDS
  rclcpp 
  tf2_ros
  geometry_msgs
  tf2
  tf2_geometry_msgs
  std_msgs
  ros2_aruco_interfaces
  mage_msgs
  nav_msgs
  rosgraph_msgs
)

# Find all dependencies
foreach(dependency IN ITEMS ${FRAME_DEMO_INCLUDE_DEPENDS})
  find_package(${dependency} REQUIRED)
endforeach()


#-----------------------------
# C++
#-----------------------------
add_executable(code src/code.cpp)
ament_target_dependencies(code ${FRAME_DEMO_INCLUDE_DEPENDS})
add_executable(listen src/listen.cpp)
ament_target_dependencies(listen ${FRAME_DEMO_INCLUDE_DEPENDS})
# add_executable(try src/try.cpp)
# ament_target_dependencies(try ${FRAME_DEMO_INCLUDE_DEPENDS})
# install(TARGETS code
#   listen
#   RUNTIME DESTINATION lib/${PROJECT_NAME}
# )


# add_executable(listener_demo src/listener_demo.cpp src/utils.cpp)
# ament_target_dependencies(listener_demo ${FRAME_DEMO_INCLUDE_DEPENDS})

# install(TARGETS listener_demo
#   RUNTIME DESTINATION lib/${PROJECT_NAME}
# )

#-----------------------------
# Installation
#-----------------------------


install(TARGETS
code
DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
listen
DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include
launch
config
  DESTINATION share/${PROJECT_NAME}/
)
ament_package()
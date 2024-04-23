cmake_minimum_required(VERSION 3.5)
# set gcc version
project(mpc_ros2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# set cmake policy -> use python?
# cmake_policy(SET CMP0148 OLD)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(casadi REQUIRED)

add_executable(mpc_ros2_node src/mpc_ros2.cpp src/main.cpp)
target_include_directories(mpc_ros2_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(
  mpc_ros2_node
  "rclcpp"
  "geometry_msgs"
  "std_msgs"
  "nav_msgs"
  "visualization_msgs"
)

target_link_libraries(mpc_ros2_node casadi)

install(TARGETS 
	mpc_ros2_node
  	DESTINATION lib/${PROJECT_NAME})
  	
add_executable(new_mpc_node src/new_mpc.cpp)
target_include_directories(new_mpc_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(
  new_mpc_node
  "rclcpp"
  "geometry_msgs"
  "std_msgs"
  "nav_msgs"
  "visualization_msgs"
)

target_link_libraries(new_mpc_node casadi)

install(TARGETS 
	new_mpc_node
  	DESTINATION lib/${PROJECT_NAME})

add_executable(new_mpc2_node src/new_mpc2.cpp)
target_include_directories(new_mpc2_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(
  new_mpc2_node
  "rclcpp"
  "geometry_msgs"
  "std_msgs"
  "nav_msgs"
  "visualization_msgs"
)

target_link_libraries(new_mpc2_node casadi)

install(TARGETS 
	new_mpc2_node
  	DESTINATION lib/${PROJECT_NAME})
  	
add_executable(new_mpc3_node src/new_mpc3.cpp)
target_include_directories(new_mpc3_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(
  new_mpc3_node
  "rclcpp"
  "geometry_msgs"
  "std_msgs"
  "nav_msgs"
  "visualization_msgs"
)

target_link_libraries(new_mpc3_node casadi)

install(TARGETS 
	new_mpc3_node
  	DESTINATION lib/${PROJECT_NAME})
  

## Trajectory
add_executable(global_path_creater src/global_path_creater.cpp src/global_main.cpp)
target_include_directories(global_path_creater PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  global_path_creater
  "rclcpp"
  "geometry_msgs"
  "nav_msgs"
)
target_link_libraries(global_path_creater)

install(TARGETS global_path_creater
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()

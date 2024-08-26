cmake_minimum_required(VERSION 3.8)
project(simplified_ur10e_moveit_plan)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(rclcpp REQUIRED)


# Function to add an executable with common settings
function(add_common_executable target_name source_file)
  add_executable(${target_name} src/${source_file})
  target_include_directories(${target_name} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_compile_features(${target_name} PUBLIC c_std_99 cxx_std_17)
  ament_target_dependencies(${target_name}
    moveit_msgs
    moveit_ros_planning_interface
    moveit_visual_tools
    rclcpp
  )
endfunction()


# Add executables
# add_common_executable(hello_moveit_v1 hello_moveit_v1.cpp)
add_common_executable(moveit_plan_node moveit_plan_node.cpp)
add_common_executable(plan_around_objects plan_around_objects.cpp)


install(TARGETS 
  moveit_plan_node
  plan_around_objects
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

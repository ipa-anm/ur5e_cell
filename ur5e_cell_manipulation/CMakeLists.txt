cmake_minimum_required(VERSION 3.8)
project(ur5e_cell_manipulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(ur_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(aruco_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(aruco_msgs REQUIRED)
find_package(color_pose_msgs REQUIRED)


###########################
#STATIC SCENARIO
###########################
add_executable(main_manipulation_static_scenario src/main_manipulation_static_scenario.cpp)
target_include_directories(main_manipulation_static_scenario PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(main_manipulation_static_scenario PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  main_manipulation_static_scenario
  "moveit_msgs"
  "moveit"
  "moveit_ros_planning"
  "ur_msgs"
  "behaviortree_cpp_v3"
  "color_pose_msgs"
  "tf2"
  "tf2_eigen"
  "tf2_geometry_msgs"
)

install(TARGETS main_manipulation_static_scenario
  DESTINATION lib/${PROJECT_NAME})



###########################
#COLOR UNIQUE SCENARIO
###########################
add_executable(main_manipulation_color_unique_scenario src/main_manipulation_color_unique_scenario.cpp)
target_include_directories(main_manipulation_color_unique_scenario PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(main_manipulation_color_unique_scenario PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  main_manipulation_color_unique_scenario
  "moveit_msgs"
  "moveit"
  "moveit_ros_planning"
  "ur_msgs"
  "behaviortree_cpp_v3"
  "color_pose_msgs"
  "tf2"
  "tf2_eigen"
  "tf2_geometry_msgs"
)

install(TARGETS main_manipulation_color_unique_scenario
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

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

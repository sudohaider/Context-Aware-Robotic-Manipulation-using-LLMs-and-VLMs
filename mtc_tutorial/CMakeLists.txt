cmake_minimum_required(VERSION 3.8)
project(mtc_tutorial)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(task_planner_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Boost REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_library(action_server SHARED
  src/mtc_action_server.cpp)
target_include_directories(action_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(action_server
  PRIVATE "MTC_TUTORIAL_BUILDING_DLL")
ament_target_dependencies(action_server
  "task_planner_interfaces"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components")
rclcpp_components_register_node(action_server PLUGIN "mtc_tutorial::MtcActionServer" EXECUTABLE mtc_action_server)
install(TARGETS
  action_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)


add_library(action_client SHARED
  src/mtc_action_client.cpp)
target_include_directories(action_client PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(action_client
  PRIVATE "MTC_TUTORIALS_CPP_BUILDING_DLL")
ament_target_dependencies(action_client
  "task_planner_interfaces"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "std_msgs")
rclcpp_components_register_node(action_client PLUGIN "mtc_tutorial::MtcActionClient" EXECUTABLE mtc_action_client)
install(TARGETS
  action_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)


add_executable(mtc_tutorial src/mtc_tutorial.cpp)
add_executable(mtc_chuppa src/mtc_chuppa.cpp)
add_executable(udp_joint_pos_sender src/udp_joint_pos_sender.cpp)

ament_target_dependencies(mtc_tutorial moveit_task_constructor_core moveit_core rclcpp)
ament_target_dependencies(mtc_chuppa moveit_task_constructor_core moveit_core rclcpp task_planner_interfaces)
ament_target_dependencies(udp_joint_pos_sender rclcpp sensor_msgs std_msgs)

target_link_libraries(udp_joint_pos_sender Boost::boost)

target_include_directories(mtc_tutorial PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(mtc_tutorial PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS mtc_tutorial mtc_chuppa
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS udp_joint_pos_sender
  DESTINATION lib/${PROJECT_NAME})

  
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  src/task_planner_service.py
  src/constraint_checker_service.py
  DESTINATION lib/${PROJECT_NAME}
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

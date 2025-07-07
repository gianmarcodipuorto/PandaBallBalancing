cmake_minimum_required(VERSION 3.8)
project(control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(uclv_robot_lib REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(realtime_tools REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(interfaces REQUIRED)
include_directories(include)

add_executable(clik src/clik.cpp)
target_link_libraries(clik Eigen3::Eigen) # Eigen is not an ament package, we have to link it manually
ament_target_dependencies(clik 
      "rclcpp" "geometry_msgs" "sensor_msgs" "uclv_robot_lib" "std_srvs" "realtime_tools" "tf2" "tf2_ros" )

add_executable(controller_pd src/controller_pd.cpp)
target_link_libraries(controller_pd Eigen3::Eigen) # Eigen is not an ament package, we have to link it manually
ament_target_dependencies(controller_pd 
      "rclcpp" "geometry_msgs" "sensor_msgs" "uclv_robot_lib" "std_srvs" "realtime_tools" "tf2" "tf2_ros" )

add_executable(handler src/handler.cpp)
ament_target_dependencies(handler 
      "rclcpp" "std_srvs" "rclcpp_action" "interfaces")

add_executable(controller_lqg src/controller_lqg.cpp)
target_link_libraries(controller_lqg Eigen3::Eigen)
ament_target_dependencies(controller_lqg 
      "rclcpp" "geometry_msgs" "sensor_msgs" "uclv_robot_lib" "std_srvs" "realtime_tools" "tf2" "tf2_ros" )


add_executable(controller_lqg_switch src/controller_lqg_switch.cpp)
target_link_libraries(controller_lqg_switch Eigen3::Eigen)
ament_target_dependencies(controller_lqg_switch 
      "rclcpp" "geometry_msgs" "sensor_msgs" "uclv_robot_lib" "std_srvs" "realtime_tools" "tf2" "tf2_ros" "interfaces" )

add_executable(home_server src/home_server.cpp)
ament_target_dependencies(home_server 
      "rclcpp"  "sensor_msgs" "std_msgs" "interfaces" "rclcpp_action")

      ########## INSTALL nodes #######################
install(TARGETS
clik controller_pd handler controller_lqg controller_lqg_switch home_server
DESTINATION lib/${PROJECT_NAME}
)


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

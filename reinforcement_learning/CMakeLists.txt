cmake_minimum_required(VERSION 3.8)
project(reinforcement_learning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_C_STANDARD 99)

# Include directory con gli header
include_directories(
  include
  include/reinforcement_learning
)

# find dependencies
find_package(std_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(realtime_tools REQUIRED)
find_package(std_srvs REQUIRED)
FIND_PACKAGE( OpenMP REQUIRED)
if(OPENMP_FOUND)
    message("OPENMP FOUND")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()


set(SOURCE_FILES
  src/callPredict.c
  src/eml_rand_mt19937ar.c
  src/eml_rand_mt19937ar_stateful.c
  src/handle.c
  src/loadRLPolicy.c
  src/mySACPolicy.c
  src/mySACPolicy_data.c
  src/mySACPolicy_initialize.c
  src/mySACPolicy_terminate.c
  src/PolicySystem.c
  src/randn.c
  src/unaryElementwise.c
)

# Nodo ROS 2
add_executable(agent
  src/agent.cpp
  ${SOURCE_FILES}
)
ament_target_dependencies(agent "rclcpp" "std_msgs" "geometry_msgs" "std_srvs")


add_executable(observation src/observation.cpp)
target_link_libraries(observation Eigen3::Eigen) 
ament_target_dependencies(observation 
      "rclcpp" "geometry_msgs" "sensor_msgs" "std_srvs")

install(TARGETS agent  observation
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


cmake_minimum_required(VERSION 3.8)
project(uclv_robot_lib)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package (Eigen3 REQUIRED NO_MODULE)

add_library(${PROJECT_NAME} 
SHARED
  src/uclv_robot_lib/Link.cpp
  src/uclv_robot_lib/links/Prismatic.cpp
  src/uclv_robot_lib/links/Revolute.cpp
  src/uclv_robot_lib/SerialLink.cpp
  src/uclv_robot_lib/robots/LBRiiwa7.cpp
  src/uclv_robot_lib/robots/MotomanSIA5F.cpp
  src/uclv_robot_lib/robots/FrankaEmikaPanda.cpp


  )

target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${EIGEN_INCLUDE_DIRS}
  )
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)
# specific order: dependents before dependencies
ament_target_dependencies(${PROJECT_NAME}
"Eigen3"
)

# Export old-style CMake variables
ament_export_include_directories("include")
ament_export_libraries(${PROJECT_NAME})

# Export modern CMake targets
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)

# specific order: dependents before dependencies
ament_export_dependencies(Eigen3)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME} 
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_generate_version_header(${PROJECT_NAME})

####

# ament_export_targets(uclv_robot_libTargets HAS_LIBRARY_TARGET)
# ament_export_dependencies(Eigen3)

# install(
#   DIRECTORY include/ ${CMAKE_CURRENT_BINARY_DIR}/include/
#   DESTINATION include/${PROJECT_NAME}
# )

# install(
#   DIRECTORY include/
#   DESTINATION include
# )
    
# install(
#   TARGETS uclv_robot_lib
#   EXPORT uclv_robot_libTargets
#   LIBRARY DESTINATION lib
#   ARCHIVE DESTINATION lib
#   RUNTIME DESTINATION bin
#   INCLUDES DESTINATION include
# )

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

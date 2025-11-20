cmake_minimum_required(VERSION 3.8)
project(arm_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)



include_directories(
  include
  include/arm_sim       # <-- aggiungi questa riga
  ${EIGEN3_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
)


# creazione eseguibile
add_executable(arm_node
  src/arm_node.cpp
  src/kinematics.cpp
  src/trajectories.cpp
)


# aggiungi le dipendenze che ament gestisce
ament_target_dependencies(arm_node rclcpp std_msgs Eigen3 visualization_msgs)

# linka manualmente sensor_msgs
target_link_libraries(arm_node ${sensor_msgs_LIBRARIES})

# include dirs pubblici
target_include_directories(arm_node PUBLIC include)

# installazione eseguibile
install(TARGETS
  arm_node
  DESTINATION lib/${PROJECT_NAME}
)

# testing e lint
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

# URDF / launch / config
install(
  DIRECTORY urdf launch config
  DESTINATION share/${PROJECT_NAME}/
)

cmake_minimum_required(VERSION 3.5)
project(unitree_sport_mode)

## Project Options
option(BUILD_EXAMPLES "Build examples" ON)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# include_directories(include)
# link_directories(src)

set (
  DEPENDENCY_LIST
  rclcpp
  unitree_go
  unitree_api
  unitree_ros2_client
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(unitree_go REQUIRED)
find_package(unitree_api REQUIRED)
find_package(unitree_ros2_client REQUIRED)

add_executable(read_sport_mode_state src/read_sport_mode_state.cpp)
target_include_directories(read_sport_mode_state PUBLIC include)
ament_target_dependencies(read_sport_mode_state ${DEPENDENCY_LIST})

add_executable(sport_mode_control src/sport_mode_control.cpp)
target_include_directories(sport_mode_control PUBLIC include)
ament_target_dependencies(sport_mode_control ${DEPENDENCY_LIST})
target_link_libraries(sport_mode_control unitree_ros2_client::unitree_ros2_client)

add_executable(trajectory_follow src/trajectory_follow_sample.cpp)
target_include_directories(trajectory_follow PUBLIC include)
ament_target_dependencies(trajectory_follow ${DEPENDENCY_LIST})
target_link_libraries(trajectory_follow unitree_ros2_client::unitree_ros2_client)

install(TARGETS
        read_sport_mode_state
        sport_mode_control
        trajectory_follow
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
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

add_executable(read_sport_mode_state src/read_sport_mode_state.cpp)
target_include_directories(read_sport_mode_state PUBLIC include)
ament_target_dependencies(read_sport_mode_state ${DEPENDENCY_LIST})

find_package(unitree_ros2_sport_client REQUIRED)

add_executable(sport_mode_control src/sport_mode_control.cpp)
target_include_directories(sport_mode_control PUBLIC include)
ament_target_dependencies(sport_mode_control ${DEPENDENCY_LIST} unitree_ros2_sport_client)
ament_target_dependencies(sport_mode_control unitree_ros2_sport_client)

install(TARGETS
        read_sport_mode_state
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
        )
add_executable(read_sport_mode_state src/read_sport_mode_state.cpp)
target_include_directories(read_sport_mode_state PUBLIC include)
ament_target_dependencies(read_sport_mode_state ${DEPENDENCY_LIST})

add_executable(sport_mode_control src/sport_mode_control.cpp)
target_include_directories(sport_mode_control PUBLIC include PUBLIC ~/unitree_ros_ws/src/unitree_ros2/example/src/include)
ament_target_dependencies(sport_mode_control ${DEPENDENCY_LIST})

install(TARGETS
        read_sport_mode_state
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
        )
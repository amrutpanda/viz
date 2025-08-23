set(LINK_LIBRARIES ${GRAPHICS_LIBRARY} ${SIMULATION_LIBRARY} 
                        ${REDIS_LIBRARY} ${DYNAMICS_LIBRARY}
                        ${RBDL_LIBRARY} ${PRIMITIVES_LIBRARY})
# include_directories(.)
include_directories(${CMAKE_CURRENT_LIST_DIR})

add_executable(dobot_controller_1 dobot_controller_1.cpp)
target_link_libraries(dobot_controller_1 ${LINK_LIBRARIES})

add_executable(dobot_controller_2 dobot_controller_2.cpp)
target_link_libraries(dobot_controller_2 ${LINK_LIBRARIES})

add_executable(multirobot_haptic_controller haptic_controller.cpp)
target_link_libraries(multirobot_haptic_controller ${LINK_LIBRARIES})

add_executable(simviz_multi_dobot simviz_multi_dobot.cpp)
target_link_libraries(simviz_multi_dobot ${LINK_LIBRARIES})

add_executable(collision_detection collisionDetection.cpp)
target_link_libraries(collision_detection ${LINK_LIBRARIES})

add_executable(state_control state_control.cpp)
target_link_libraries(state_control ${LINK_LIBRARIES})

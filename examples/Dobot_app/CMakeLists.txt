set(LINK_LIBRARIES ${GRAPHICS_LIBRARY} ${SIMULATION_LIBRARY} 
                        ${REDIS_LIBRARY} ${DYNAMICS_LIBRARY}
                        ${RBDL_LIBRARY} ${PRIMITIVES_LIBRARY})
# include_directories(.)
include_directories(${CMAKE_CURRENT_LIST_DIR})

add_executable(dobot_controller dobot_controller.cpp)
target_link_libraries(dobot_controller ${LINK_LIBRARIES})

add_executable(haptic_controller haptic_controller.cpp)
target_link_libraries(haptic_controller ${LINK_LIBRARIES})

add_executable(simviz_dobot simviz_dobot.cpp)
target_link_libraries(simviz_dobot ${LINK_LIBRARIES})

add_executable(state_control state_control.cpp)
target_link_libraries(state_control ${LINK_LIBRARIES})
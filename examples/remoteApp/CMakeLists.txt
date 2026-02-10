set(LINK_LIBRARIES ${GRAPHICS_LIBRARY} ${SIMULATION_LIBRARY} 
                        ${REDIS_LIBRARY} ${DYNAMICS_LIBRARY}
                        ${RBDL_LIBRARY} ${PRIMITIVES_LIBRARY})
# include_directories(.)
include_directories(${CMAKE_CURRENT_LIST_DIR})

add_executable(robot_controller_rto robot_controller_rto.cpp)
target_link_libraries(robot_controller_rto ${LINK_LIBRARIES})

add_executable(simviz_rto simviz_dobot_rto.cpp)
target_link_libraries(simviz_rto ${LINK_LIBRARIES})

add_executable(simviz_button simviz_button.cpp)
target_link_libraries(simviz_button ${LINK_LIBRARIES})
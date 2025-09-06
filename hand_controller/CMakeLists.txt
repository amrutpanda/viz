# include_directories(${CMAKE_CURRENT_LIST_DIR})

set(LINK_LIBRARIES ${GRAPHICS_LIBRARY} ${SIMULATION_LIBRARY} 
                        ${REDIS_LIBRARY} ${DYNAMICS_LIBRARY}
                        ${RBDL_LIBRARY} ${PRIMITIVES_LIBRARY})

add_executable(simviz_handcontroller src/simviz.cpp)
target_link_libraries(simviz_handcontroller ${LINK_LIBRARIES})

add_executable(Inverse_kinematics src/Inverse_kinematics.cpp)
target_link_libraries(Inverse_kinematics ${LINK_LIBRARIES})


add_executable(simviz_franka src/simviz_franka.cpp)
target_link_libraries(simviz_franka ${LINK_LIBRARIES})
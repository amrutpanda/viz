# include_directories(${CMAKE_CURRENT_LIST_DIR})

set(LINK_LIBRARIES ${GRAPHICS_LIBRARY} ${SIMULATION_LIBRARY} 
                        ${REDIS_LIBRARY} ${DYNAMICS_LIBRARY}
                        ${RBDL_LIBRARY} ${PRIMITIVES_LIBRARY})

add_executable(simviz_handcontroller src/simviz.cpp)
target_link_libraries(simviz_handcontroller ${LINK_LIBRARIES})
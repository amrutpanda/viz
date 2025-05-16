# add_executable(rigidbodyTest ${PROJECT_SOURCE_DIR}/examples/rigidbodyTest.cpp)
set(LINK_LIBRARIES ${GRAPHICS_LIBRARY} ${SIMULATION_LIBRARY} 
                        ${REDIS_LIBRARY} ${DYNAMICS_LIBRARY}
                        ${RBDL_LIBRARY})
include_directories(.)

add_executable(rigidbodyTest rigidbody_test.cpp)
target_link_libraries(rigidbodyTest ${LINK_LIBRARIES})

add_executable(redis_test redis_test.cpp)
target_link_libraries(redis_test ${LINK_LIBRARIES})

add_executable(simviz_1 simviz_1.cpp)
target_link_libraries(simviz_1 ${LINK_LIBRARIES})

add_executable(robot_controller robot_controller.cpp)
target_link_libraries(robot_controller ${LINK_LIBRARIES})
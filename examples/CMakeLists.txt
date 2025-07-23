# add_executable(rigidbodyTest ${PROJECT_SOURCE_DIR}/examples/rigidbodyTest.cpp)
set(LINK_LIBRARIES ${GRAPHICS_LIBRARY} ${SIMULATION_LIBRARY} 
                        ${REDIS_LIBRARY} ${DYNAMICS_LIBRARY}
                        ${RBDL_LIBRARY} ${PRIMITIVES_LIBRARY})
# include_directories(.)
include_directories(${CMAKE_CURRENT_LIST_DIR})

add_executable(rigidbodyTest rigidbody_test.cpp)
target_link_libraries(rigidbodyTest ${LINK_LIBRARIES})

# add_executable(redis_test redis_test.cpp)
# target_link_libraries(redis_test ${LINK_LIBRARIES})

add_executable(simviz_1 simviz_1.cpp)
target_link_libraries(simviz_1 ${LINK_LIBRARIES})

add_executable(robot_controller robot_controller.cpp)
target_link_libraries(robot_controller ${LINK_LIBRARIES})

# add_executable(haptic_controller haptic_controller.cpp)
# target_link_libraries(haptic_controller ${LINK_LIBRARIES})

add_subdirectory(Dobot_app)

# add_executable(jogging_controller ${CMAKE_CURRENT_LIST_DIR}/Keyboard_jogging/jogging_controller.cpp)
# target_link_libraries(jogging_controller ${LINK_LIBRARIES})

# add_executable(kbdReader ${CMAKE_CURRENT_LIST_DIR}/Keyboard_jogging/keyboardReader.cpp)
# target_link_libraries(kbdReader ${LINK_LIBRARIES})

# add_executable(simviz_jogging ${CMAKE_CURRENT_LIST_DIR}/Keyboard_jogging/simviz_jogging.cpp)
# target_link_libraries(simviz_jogging ${LINK_LIBRARIES})
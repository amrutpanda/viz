# add_executable(rigidbodyTest ${PROJECT_SOURCE_DIR}/examples/rigidbodyTest.cpp)
set(LINK_LIBRARIES ${GRAPHICS_LIBRARY} ${SIMULATION_LIBRARY})
add_executable(rigidbodyTest rigidbody_test.cpp)
target_link_libraries(rigidbodyTest ${LINK_LIBRARIES})
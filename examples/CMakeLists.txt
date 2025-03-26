# add_executable(rigidbodyTest ${PROJECT_SOURCE_DIR}/examples/rigidbodyTest.cpp)
add_executable(rigidbodyTest rigidbody_test.cpp)
target_link_libraries(rigidbodyTest ${LIBRARIES})
message("${PROJECT_SOURCE_DIR}")
add_library(simulation sim.cpp ${PROJECT_SOURCE_DIR}/src/Logger.cpp)
add_executable(sim sim_test.cpp)
target_link_libraries(sim ${LIBRAREIS} simulation  -lBulletDynamics -lBulletCollision -lLinearMath  )
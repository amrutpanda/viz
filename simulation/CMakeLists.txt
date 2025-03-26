message("${PROJECT_SOURCE_DIR}")
add_library(simulation sim.cpp ${PROJECT_SOURCE_DIR}/src/Logger.cpp)
target_link_libraries(simulation -lBulletDynamics -lBulletCollision -lLinearMath )
set(SIMULATION_LIBRARY simulation CACHE INTERNAL "")

add_executable(sim sim_test.cpp)
target_link_libraries(sim ${SIMULATION_LIBRARY})


# add_executable(sim sim_test.cpp)
# target_link_libraries(sim ${LIBRAREIS} simulation  -lBulletDynamics -lBulletCollision -lLinearMath  )
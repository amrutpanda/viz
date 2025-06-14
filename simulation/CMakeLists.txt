message("${PROJECT_SOURCE_DIR}")

find_package(assimp REQUIRED)
message("${ASSIMP_LIBRARIES}")
include_directories(${ASSIMP_INCLUDE_DIRS} ${CMAKE_CURRENT_LIST_DIR})

add_library(simulation sim.cpp simMultiBody.cpp simDebugViewer.cpp
                        BulletURDFImporter.cpp ForceSensor.cpp ${PROJECT_SOURCE_DIR}/src/Logger.cpp)
target_link_libraries(simulation -lBulletDynamics -lBulletCollision -lLinearMath 
                            ${ASSIMP_LIBRARIES} ${urdfdom_LIBRARIES} ${GRAPHICS_LIBRARY})
set(SIMULATION_LIBRARY simulation CACHE INTERNAL "")

add_executable(sim sim_test.cpp)
target_link_libraries(sim ${SIMULATION_LIBRARY})


# add_executable(sim sim_test.cpp)
# target_link_libraries(sim ${LIBRAREIS} simulation  -lBulletDynamics -lBulletCollision -lLinearMath  )
# chai3d library.
list(APPEND CMAKE_PREFIX_PATH "/home/merai/Files/C++/chai3d/build")
find_package(chai3d REQUIRED)
include_directories(${CHAI3D_INCLUDE_DIRS})
link_directories(${CHAI3D_LIBRARY_DIRS})

set(LINK_LIBRARIES ${GRAPHICS_LIBRARY} ${SIMULATION_LIBRARY} 
                        ${REDIS_LIBRARY} ${DYNAMICS_LIBRARY}
                        ${RBDL_LIBRARY} ${PRIMITIVES_LIBRARY}
                        ${CHAI3D_LIBRARIES})
message("${CHAI3D_LIBRARY_DIRS}")
include_directories(${CMAKE_CURRENT_LIST_DIR})
add_executable(chai_haptics_redis_driver chai_haptics_redis_driver.cpp)
target_link_libraries(chai_haptics_redis_driver ${LINK_LIBRARIES})

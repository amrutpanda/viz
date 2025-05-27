add_subdirectory(ruckig)

set(LINK_LIBRARIES ${DYNAMICS_LIBRARY})

message("Dynamics lib: ${LINK_LIBRARIES}")
include_directories(${PROJECT_SOURCE_DIR}/primitives)

add_library(primitives SHARED JointTask.cpp)
target_link_libraries(primitives ${LINK_LIBRARIES})

set(PRIMITIVES_LIBRARY primitives CACHE INTERNAL "")
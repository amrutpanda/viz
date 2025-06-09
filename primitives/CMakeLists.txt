set(BUILD_TESTS OFF CACHE BOOL "Disable Ruckig tests")
set(BUILD_EXAMPLES OFF CACHE BOOL "Disable Ruckig examples")
set(BUILD_CLOUD_CLIENT CACHE BOOL "Disable Rucking Pro")
add_subdirectory(ruckig)
include_directories(ruckig/include)

set(LINK_LIBRARIES ${DYNAMICS_LIBRARY})
include_directories(${PROJECT_SOURCE_DIR}/primitives)

add_library(primitives SHARED JointTask.cpp HapticController.cpp)
target_link_libraries(primitives ${LINK_LIBRARIES} ruckig)
target_include_directories(primitives PUBLIC ${PROJECT_SOURCE_DIR}/primitives)

set(PRIMITIVES_LIBRARY primitives CACHE INTERNAL "")
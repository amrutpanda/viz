set(LINK_LIBRARIES ${RBDL_LIBRARY})

message("rbdl lib: ${LINK_LIBRARIES}")
include_directories(.)

add_library(dynamics STATIC dynamics_model.cpp)
target_link_libraries(dynamics ${LINK_LIBRARIES})

set(DYNAMIC_MODEL dynamics CACHE INTERNAL "")
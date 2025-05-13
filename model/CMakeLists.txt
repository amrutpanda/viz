set(LINK_LIBRARIES ${RBDL_LIBRARY})

message("rbdl lib: ${LINK_LIBRARIES}")
include_directories(${PROJECT_SOURCE_DIR}/model)

add_library(dynamics SHARED dynamics_model.cpp)
target_link_libraries(dynamics ${LINK_LIBRARIES})

set(DYNAMICS_LIBRARY dynamics CACHE INTERNAL "")
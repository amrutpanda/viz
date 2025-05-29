set(LINK_LIBRARIES ${RBDL_LIBRARY} ${RBDL_URDFREADER_LIBRARY})

message("rbdl lib: ${LINK_LIBRARIES}")
include_directories(${PROJECT_SOURCE_DIR}/model)

add_library(dynamics SHARED dynamics_model.cpp)
target_link_libraries(dynamics ${LINK_LIBRARIES})
target_include_directories(dynamics PUBLIC ${PROJECT_SOURCE_DIR}/model)

set(DYNAMICS_LIBRARY dynamics CACHE INTERNAL "")
# list(APPEND CMAKE_PREFIX_PATH "/home/asp/Files/cpp/projects/bullet3/install/lib/cmake/bullet/")
find_package(Bullet REQUIRED PATHS "/home/asp/Files/cpp/projects/bullet3/install/lib/cmake/bullet/")
message("bullet libs: ${BULLET_LIBRARIES}")
message("bullet DIR: ${BULLET_INCLUDE_DIRS}")


set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

# include_directories(${BULLET_INCLUDE_DIRS})
# include_directories("/home/asp/Files/cpp/projects/bullet3/install/include/")
include_directories("/home/asp/Files/cpp/projects/bullet3/install/include/bullet")
# link_directories(${BULLET_LIBRARY_DIRS})
link_directories("/home/asp/Files/cpp/projects/bullet3/install/lib/")
message(LIBS: ,"${LIBS}")

add_executable(sim sim.cpp)
target_link_libraries(sim ${BULLET_LIBRARIES} ${LIBS})
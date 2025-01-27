cmake_minimum_required(VERSION 3.15)
project(viz)

set(CMAKE_PREFIX_PATH "/home/asp/Files/cpp/projects/ogre/install/lib/OGRE/cmake/")
find_package(OGRE REQUIRED)
# list(APPEND CMAKE_PREFIX_PATH "/opt/ros/humble/lib/x86_64-linux-gnu/urdfdom_headers/cmake/urdfdom_headers-config.cmake")
list(APPEND CMAKE_PREFIX_PATH "/opt/ros/humble/lib/x86_64-linux-gnu/urdfdom_headers/cmake/")
find_package(urdfdom_headers)
list(APPEND CMAKE_PREFIX_PATH "/home/asp/Files/cpp/projects/urdfdom/install/lib/urdfdom/cmake/")
find_package(urdfdom REQUIRED)
include_directories(${urdfdom_INCLUDE_DIRS})

# SDL2 library
find_package(SDL2 REQUIRED)
include_directories(${SDL2_INCLUDE_DIRS})

list(APPEND CMAKE_PREFIX_PATH "/home/asp/Files/cpp/projects/rbdl/install/lib/cmake/RBDL/")
set(CUSTOM_RBDL_PATH  "/home/asp/Files/cpp/projects/rbdl/install" CACHE INTERNAL "rbdl path")
find_package(RBDL REQUIRED)
include_directories(${RBDL_INCLUDE_DIR})


set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
include_directories(${PROJECT_SOURCE_DIR}/include)

add_executable(app src/main.cpp src/mGraphics.cpp src/mRobot.cpp
                    src/mObject.cpp)
add_executable(test_app src/mainloop.cpp src/mGraphics.cpp src/mRobot.cpp
                    src/mObject.cpp)
add_executable(rbdl_test src/rbdl_test.cpp)

message("STATUS ${RBDL_URDFREADER_LIBRARY}")

target_link_libraries(rbdl_test ${RBDL_LIBRARY} ${RBDL_URDFREADER_LIBRARY})
target_link_libraries(app ${OGRE_LIBRARIES} ${urdfdom_LIBRARIES} ${SDL2_LIBRARIES})
target_link_libraries(test_app ${OGRE_LIBRARIES} ${urdfdom_LIBRARIES} ${SDL2_LIBRARIES})

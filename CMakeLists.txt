cmake_minimum_required(VERSION 3.15)
project(viz)

function(fetch_ext_library _lib_name github_link git_tag )
    include(FetchContent)
    FetchContent_Declare(
        ${_lib_name}
        GIT_REPOSITORY ${github_link}
        GIT_TAG        ${git_tag}
        CMAKE_ARGS     ""
        BUILD_COMMAND  ""
        INSTALL COMMAND ""
    )
    FetchContent_MakeAvailable(${_lib_name})   
endfunction()


set(CMAKE_PREFIX_PATH "/home/asp/Files/cpp/projects/ogre/install/lib/OGRE/cmake/")
find_package(OGRE REQUIRED)
# list(APPEND CMAKE_PREFIX_PATH "/opt/ros/humble/lib/x86_64-linux-gnu/urdfdom_headers/cmake/urdfdom_headers-config.cmake")
list(APPEND CMAKE_PREFIX_PATH "/opt/ros/humble/lib/x86_64-linux-gnu/urdfdom_headers/cmake/")
find_package(urdfdom_headers)
list(APPEND CMAKE_PREFIX_PATH "/home/asp/Files/cpp/projects/urdfdom/install/lib/urdfdom/cmake/")
find_package(urdfdom REQUIRED)
include_directories(${urdfdom_INCLUDE_DIRS} ${OGRE_INCLUDE_DIRS})

# SDL2 library
find_package(SDL2 REQUIRED)
include_directories(${SDL2_INCLUDE_DIRS})

list(APPEND CMAKE_PREFIX_PATH "/home/asp/Files/cpp/projects/rbdl/install/lib/cmake/RBDL/")
set(CUSTOM_RBDL_PATH  "/home/asp/Files/cpp/projects/rbdl/install" CACHE INTERNAL "rbdl path")
find_package(RBDL REQUIRED)
include_directories(${RBDL_INCLUDE_DIR})

# list(APPEND CMAKE_PREFIX_PATH "/home/asp/Files/cpp/projects/bullet3/install/lib/cmake/bullet/")
# find_package(Bullet REQUIRED PATHS "/home/asp/Files/cpp/projects/bullet3/install/lib/cmake/bullet/")
# find_package(Bullet REQUIRED)
# if(${Bullet_FOUND})
#     message("Found Bullet ${BULLET_INCLUDE_DIRS}")
#     message("Bullet VERSION:  ${BULLET_LIBRARIES}")
#     include_directories(${BULLET_INCLUDE_DIRS})
# else()
#     message("I am here")
#     set(BULLET_INCLUDE_DIR "/home/asp/Files/cpp/projects/bullet3/install/include/bullet")
#     # set(BULLET_LIBRARIES "/home/asp/Files/cpp/projects/bullet3/install/")
#     # find_library(BULLET_LIBRARIES "/home/asp/Files/cpp/projects/bullet3/install/lib")
#     link_directories("/home/asp/Files/cpp/projects/bullet3/install/lib")
#     include_directories(${BULLET_INCLUDE_DIR})
#     message("Bullet include dir: ${BULLET_INCLUDE_DIR} \n Bullet library dir: ${BULLET_LIBRARIES}")
# endif()

# testing fetchcontent

include(FetchContent)
set(pugixml_VERSION 1.15)
set(FETCHCONTENT_QUIET FALSE)
message("Downloading ...")
FetchContent_Declare(
    pugixml
    GIT_REPOSITORY https://github.com/zeux/pugixml.git
    GIT_TAG        v1.15
    BUILD_COMMAND  ""
    )
FetchContent_MakeAvailable(pugixml)

set(BULLET_INCLUDE_DIR "/home/asp/Files/cpp/projects/bullet3/install/include/bullet/")
set(BULLET_LIB "/home/asp/Files/cpp/projects/bullet3/install/lib")
include_directories(${BULLET_INCLUDE_DIR} ${PROJECT_SOURCE_DIR}/simulation ${PROJECT_SOURCE_DIR}/include)
link_directories(${BULLET_LIB})

set(LIBRARIES ${OGRE_LIBRARIES} ${urdfdom_LIBRARIES} ${SDL2_LIBRARIES} pugixml)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

# add_library(graphics src/mGraphics.cpp src/mRobot.cpp src/dmObject.cpp src/mObject.cpp
#                         src/axis.cpp src/mCamera.cpp)                   

# add_executable(test_app src/mainloop.cpp)
# add_executable(Imgui_test src/ImguiTest.cpp)

add_executable(test_app src/mainloop.cpp src/mGraphics.cpp src/mRobot.cpp 
                    src/dmObject.cpp src/mObject.cpp src/axis.cpp src/mCamera.cpp)
add_executable(Imgui_test src/ImguiTest.cpp src/mGraphics.cpp src/mRobot.cpp 
                    src/dmObject.cpp src/mObject.cpp src/axis.cpp src/mCamera.cpp)
# add_executable(rbdl_test src/rbdl_test.cpp)

# target_link_libraries(rbdl_test ${RBDL_LIBRARY} ${RBDL_URDFREADER_LIBRARY})

target_link_libraries(test_app ${LIBRARIES})
target_link_libraries(Imgui_test ${LIBRARIES})
# target_link_libraries(sim ${LIBRAREIS})

# add_subdirectory(simulation)
# add_subdirectory(examples)


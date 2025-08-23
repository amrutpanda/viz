cmake_minimum_required(VERSION 3.15)
project(viz)
set(CMAKE_BUILD_TYPE "Debug")
set(CXX_STANDARD_REQUIRED 20)

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


set(CMAKE_PREFIX_PATH "/home/merai/Files/C++/ogre/install/lib/OGRE/cmake/")
find_package(OGRE REQUIRED)

# list(APPEND CMAKE_PREFIX_PATH "/opt/ros/humble/lib/x86_64-linux-gnu/urdfdom_headers/cmake/urdfdom_headers-config.cmake")

list(APPEND CMAKE_PREFIX_PATH "/home/merai/Files/C++/urdfdom_headers/install/lib/urdfdom_headers/cmake/")
find_package(urdfdom_headers)

list(APPEND CMAKE_PREFIX_PATH "/home/merai/Files/C++/console_bridge/install/lib/console_bridge/cmake/")
find_package(console_bridge REQUIRED)

list(APPEND CMAKE_PREFIX_PATH "/home/merai/Files/C++/urdfdom/install/lib/urdfdom/cmake/")
find_package(urdfdom REQUIRED)
include_directories(${urdfdom_INCLUDE_DIRS} ${console_bridge_INCLUDE_DIRS} ${OGRE_INCLUDE_DIRS})

# SDL2 library
find_package(SDL2 REQUIRED)
include_directories(${SDL2_INCLUDE_DIRS})

list(APPEND CMAKE_PREFIX_PATH "/home/merai/Files/C++/rbdl/install/lib/cmake/RBDL/")
set(CUSTOM_RBDL_PATH  "/home/merai/Files/C++/rbdl/install/" CACHE INTERNAL "rbdl path")
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
set(THIRD_PARTY_INSTALL_DIR "${PROJECT_SOURCE_DIR}/DEPS")
message("Downloading ...")
# 1. Fetch pugixml package.
FetchContent_Declare(
    pugixml
    GIT_REPOSITORY https://github.com/zeux/pugixml.git
    GIT_TAG        v1.15
    BUILD_COMMAND  ""
    )
FetchContent_MakeAvailable(pugixml)

# # include(${PROJECT_SOURCE_DIR}/cmake/build_urdfdom.cmake)

set(BULLET_INCLUDE_DIR "/home/merai/Files/C++/bullet3/install/include/bullet/")
set(BULLET_LIB "/home/merai/Files/C++/bullet3/install/lib")
include_directories(${BULLET_INCLUDE_DIR} ${PROJECT_SOURCE_DIR}/simulation ${PROJECT_SOURCE_DIR}/include)
link_directories(${BULLET_LIB})

# need extra care here. Console_bridge library has been added.
set(LIBRARIES ${OGRE_LIBRARIES} ${urdfdom_LIBRARIES}
               ${RBDL_LIBRARY} ${RBDL_URDFREADER_LIBRARY}
                ${SDL2_LIBRARIES} pugixml ${console_bridge_LIBRARIES})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
                 
get_target_property(CONSOLE_BRIDGE_PATH console_bridge::console_bridge LOCATION)
message("CCC ${CONSOLE_BRIDGE_PATH}")

# add_executable(test_app src/mainloop.cpp src/mGraphics.cpp src/mRobot.cpp 
#                     src/dmObject.cpp src/mObject.cpp src/axis.cpp src/mCamera.cpp)
# add_executable(Imgui_test src/ImguiTest.cpp src/mGraphics.cpp src/mRobot.cpp 
#                     src/dmObject.cpp src/mObject.cpp src/axis.cpp src/mCamera.cpp)




# add_executable(rbdl_test src/rbdl_test.cpp)
# target_link_libraries(rbdl_test ${RBDL_LIBRARY} ${RBDL_URDFREADER_LIBRARY})

# target_link_libraries(test_app ${LIBRARIES})
# target_link_libraries(Imgui_test ${LIBRARIES})

find_library(HIREDIS_LIB NAMES hiredis)
add_library(redisclient STATIC ${PROJECT_SOURCE_DIR}/src/redisclient.cpp)
target_link_libraries(redisclient ${HIREDIS_LIB})
set(REDIS_LIBRARY redisclient CACHE INTERNAL "")

include_directories(model)

add_subdirectory(src)
add_subdirectory(simulation)
add_subdirectory(model)
add_subdirectory(primitives)
add_subdirectory(examples)
add_subdirectory(Haptics)
add_subdirectory(hand_controller)

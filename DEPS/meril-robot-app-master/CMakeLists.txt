cmake_minimum_required(VERSION 3.16)
project(meril-robot)

message(STATUS "Using c++20")
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})
#find_package(Eigen3 3.3 REQUIRED NO_MODULE)

set(target_name ${PROJECT_NAME})

include(CheckSanitizer)
CheckSanitizer()
if (NOT MCX_SANITIZER STREQUAL "none")
    set(MCX_SANITIZER_FLAGS "-fsanitize=${MCX_SANITIZER}")
endif ()

include(GitVersion)
#GitVersion()

# Set a default install prefix
# Set a default build type if none was specified
set(default_build_type "Release")
if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
    set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE
            STRING "Choose the type of build." FORCE)
    # Set the possible values of build type for cmake-gui
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
            "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif ()

# options
option(BUILD_SHARED_LIBS "Build using shared libraries" ON)
option(INSTALL_DEV_KIT "Install development headers and a library" ON)
option(ADD_LTO "Add LTO" OFF)
option(ADD_FE "Add floating point exceptions" OFF)

if (ADD_FE)
    set(compile_defs ${compile_defs} -DADD_FE)
endif ()

# add include directories
set(include_dirs ${include_dirs}
        ${CMAKE_CURRENT_SOURCE_DIR}/app
        ${CMAKE_CURRENT_SOURCE_DIR}/control
        ${CMAKE_CURRENT_SOURCE_DIR}/logic
        ${CMAKE_CURRENT_SOURCE_DIR})

# add libraries
set(linking_libs ${linking_libs}
        mcx-math
        mcx-core
        mcx-control3
        mcx-motion
        mcx-mechanics
        mcx-udp
)

# checking type of the configuration
# Setting common flags
string(TOLOWER "${CMAKE_BUILD_TYPE}" CMAKE_BUILD_TYPE_LOWER)
set(compile_defs ${compile_defs} -DENABLE_ASSERTS -DLOG_LEVEL_INFO -DLOG_LEVEL_SM
        -DMOTORCORTEX_ROBOT_VERSION_MAJOR=${MCX_VERSION_MAJOR}
        -DMOTORCORTEX_ROBOT_VERSION=${MCX_VERSION_FULL}_${CMAKE_BUILD_TYPE_LOWER})

if (CMAKE_BUILD_TYPE_LOWER STREQUAL "debug")
    set(compile_defs ${compile_defs} -Wall -DLOG_LEVEL_DEBUG)
    message(STATUS "Debug configuration flags: ${compile_defs}")
else ()
    set(compile_defs ${compile_defs} -Wall -Werror)
    message(STATUS "Release configuration flags: ${compile_defs}")
endif ()

# sub-directories
add_subdirectory(app)
add_subdirectory(logic)
add_subdirectory(control)
add_subdirectory(instrument)

add_executable(${target_name} app_main.cpp ${src_files})
if (ADD_LTO)
    include(CheckIPOSupported)
    check_ipo_supported(RESULT supported OUTPUT error)
    if (supported)
        message(STATUS "IPO / LTO enabled")
        set_property(TARGET ${target_name} PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE)
    else ()
        message(STATUS "IPO / LTO not supported: <${error}>")
    endif ()
endif ()
add_definitions(${compile_defs})
target_include_directories(${target_name} PRIVATE ${include_dirs})
target_link_libraries(${target_name} PRIVATE ${linking_libs} ${MCX_SANITIZER_FLAGS} -lm -lmvec)
target_compile_options(${target_name} PRIVATE ${compile_flags} ${MCX_SANITIZER_FLAGS})

install(TARGETS ${target_name} DESTINATION bin)

#####Configs#####
include(ConfigZip)
#ConfigZip(${CMAKE_CURRENT_SOURCE_DIR}/config ${MCX_VERSION_LITE})

#####CPack#####
set(CPACK_PACKAGE_NAME ${PROJECT_NAME})
set(CPACK_PACKAGE_VENDOR ${VECTIONEER.})
set(CPACK_PACKAGE_VERSION ${MCX_VERSION_LITE})
set(CPACK_PACKAGING_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX})
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Meril Robot Template.\n "
        "Meril Robot Template is an example of a manipulator control application.")
set(CPACK_PACKAGE_CONTACT "info@vectioneer.com")
set(CPACK_DEBIAN_PACKAGE_HOMEPAGE "http://www.vectioneer.com")
set(CPACK_DEBIAN_PACKAGE_REPLACES ${PROJECT_NAME})
set(CPACK_GENERATOR "DEB")
set(CPACK_SOURCE_GENERATOR "TBZ2;TGZ;ZIP")
set(CPACK_SOURCE_IGNORE_FILES "/build/;/.git/;~$;${CPACK_SOURCE_IGNORE_FILES}")
set(CPACK_SOURCE_PACKAGE_FILE_NAME "${PROJECT_NAME}-${MCX_VERSION_LITE}")
set(CPACK_DEBIAN_PACKAGE_DEPENDS "motorcortex-core (>= 2.18.0),
                                  motorcortex-math (>= 2.8.1),
                                  motorcortex-control3 (>= 3.12.0),
                                  motorcortex-motion (>= 3.2.0),
                                  motorcortex-mechanics (>= 1.10.0),
                                  motorcortex-io (>=2.7.0)")
add_custom_target(deb-${PROJECT_NAME} COMMAND ${CMAKE_CPACK_COMMAND} package_source)
#add_custom_target(dist-${PROJECT_NAME} COMMAND ${CMAKE_MAKE_PROGRAM} package_source)
include(CPack)


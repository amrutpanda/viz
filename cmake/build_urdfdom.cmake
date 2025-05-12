include(FetchContent)
# 1. Fetch urdfdom_headers.
FetchContent_Declare(
	urdfdom_headers
	GIT_REPOSITORY https://github.com/ros/urdfdom_headers.git
	GIT_TAG 1.0.6
	BUILD_COMMAND ""
)
FetchContent_GetProperties(urdfdom_headers)
if (NOT urdfdom_headers_POPULATED)
	FetchContent_Populate(urdfdom_headers)
	execute_process(
		COMMAND ${CMAKE_COMMAND} -S ${urdfdom_headers_SOURCE_DIR} -B ${urdfdom_headers_BINARY_DIR}
		        -DCMAKE_INSTALL_PREFIX=${THIRD_PARTY_INSTALL_DIR}/urdfdom_headers
		RESULT_VARIABLE res_configure
  	)
	if(NOT res_configure EQUAL 0)
		message(FATAL_ERROR "Failed to configure urdfdom_headers")
	endif()
	
	execute_process(
		COMMAND ${CMAKE_COMMAND} --build ${urdfdom_headers_BINARY_DIR} --target install
		RESULT_VARIABLE res_build
	)
	
	if(NOT res_build EQUAL 0)
		message(FATAL_ERROR "Failed to build/install urdfdom_headers")
	endif()
	
endif()
list(APPEND CMAKE_PREFIX_PATH "${THIRD_PARTY_INSTALL_DIR}/urdfdom_headers")
list(APPEND CMAKE_PREFIX_PATH "${THIRD_PARTY_INSTALL_DIR}/urdfdom_headers/lib/urdfdom_headers/cmake")
# list(APPEND CMAKE_PREFIX_PATH "/home/amrut/Files/C++/viz/DEPS/urdfdom_headers/lib/urdfdom_headers/cmake/")

# 2. Fetch Console_bridge package.
FetchContent_Declare(
  console_bridge
  GIT_REPOSITORY https://github.com/ros/console_bridge.git
  GIT_TAG        1.0.2
)
FetchContent_GetProperties(console_bridge)
if (NOT console_bridge_POPULATED)
	FetchContent_POPULATE(console_bridge)
	execute_process(
		COMMAND ${CMAKE_COMMAND} -S ${console_bridge_SOURCE_DIR} -B ${console_bridge_BINARY_DIR}
		        -DCMAKE_INSTALL_PREFIX=${THIRD_PARTY_INSTALL_DIR}/console_bridge
		RESULT_VARIABLE res_configure
  	)
  	message("${res_configure}")
	if (NOT res_configure EQUAL 0)
		message(FATAL_ERROR "Failed to configure console_bridge")
	endif()
	
	execute_process(
		COMMAND ${CMAKE_COMMAND} --build ${console_bridge_BINARY_DIR} --target install
		RESULT_VARIABLE res_build
	)
	
	if (NOT res_build EQUAL 0)
		message(FATAL_ERROR "Failed to build console_bridge")
	endif()
endif()
list(APPEND CMAKE_PREFIX_PATH "${THIRD_PARTY_INSTALL_DIR}/console_bridge")
list(APPEND CMAKE_PREFIX_PATH "${THIRD_PARTY_INSTALL_DIR}/console_bridge/lib/console_bridge/cmake")
message(STATUS "Final CMAKE_PREFIX_PATH: ${CMAKE_PREFIX_PATH}")
message(("hello"))
# 3. fetch urdfdom package.
FetchContent_Declare(
	urdfdom
	GIT_REPOSITORY https://github.com/ros/urdfdom.git
	GIT_TAG 4.0.1
	BUILD_COMMAND ""
)
# FetchContent_MakeAvailable(urdfdom)
# list(APPEND CMAKE_PREFIX_PATH "/home/amrut/Files/C++/viz/DEPS/urdfdom_headers/lib/urdfdom_headers/cmake/")
list(JOIN CMAKE_PREFIX_PATH ";" joined_prefix_path)
FetchContent_GetProperties(urdfdom)
if (NOT urdfdom_POPULATED)
	FetchContent_POPULATE(urdfdom)
	execute_process(
		COMMAND ${CMAKE_COMMAND} -S ${urdfdom_SOURCE_DIR} -B ${urdfdom_BINARY_DIR} 
                -DCMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}
		        -DCMAKE_INSTALL_PREFIX=${THIRD_PARTY_INSTALL_DIR}/urdfdom
		RESULT_VARIABLE res_configure
        ERROR_VARIABLE err
  	)
  	message("${res_configure}")
    message(("err = ${err}"))
	if (NOT res_configure EQUAL 0)
		message(FATAL_ERROR "Failed to configure urdfdom")
	endif()
	
	execute_process(
		COMMAND ${CMAKE_COMMAND} --build ${urdfdom_BINARY_DIR} --target install
		RESULT_VARIABLE res_build
	)
	
	if (NOT res_build EQUAL 0)
		message(FATAL_ERROR "Failed to build urdfdom")
	endif()
endif()
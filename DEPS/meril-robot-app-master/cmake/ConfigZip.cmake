
macro (ConfigZip directory version)

  file(GLOB children RELATIVE ${directory} ${directory}/*)
  foreach(child ${children})
    if(IS_DIRECTORY ${directory}/${child} AND
       NOT IS_SYMLINK ${directory}/${child} AND
       EXISTS ${directory}/${child}/.mcx-default-config)

      set(package_name ${PROJECT_NAME}-config-${child})
      add_custom_target(${package_name} ALL COMMAND
                          ${CMAKE_COMMAND} -E tar "czf"
                          "${CMAKE_CURRENT_BINARY_DIR}/${package_name}-${version}.tar.gz"
                          --format=gnutar ${child}/* # ${child}/* doesn't add files starting with .
                          WORKING_DIRECTORY ${directory})

    endif()
  endforeach()

endmacro ()

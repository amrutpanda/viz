project(meril-robot)

file(GLOB src
        "*.cpp"
)

file(GLOB inc
        "*.h"
)

set(src_files ${src_files} ${src} CACHE INTERNAL "")
set(inc_files ${inc_files} ${inc} CACHE INTERNAL "")

add_subdirectory(meril)
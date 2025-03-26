add_library(graphics mObject.cpp dmObject.cpp axis.cpp mCamera.cpp mRobot.cpp mGraphics.cpp)
target_link_libraries(graphics ${LIBRARIES})
set(GRAPHICS_LIBRARY graphics CACHE INTERNAL "")

add_executable(test_app mainloop.cpp)
target_link_libraries(test_app ${GRAPHICS_LIBRARY})
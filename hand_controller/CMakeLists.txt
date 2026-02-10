# include_directories(${CMAKE_CURRENT_LIST_DIR})

include_directories(${CMAKE_CURRENT_LIST_DIR})
include_directories(.)

set(LINK_LIBRARIES ${GRAPHICS_LIBRARY} ${SIMULATION_LIBRARY} 
                        ${REDIS_LIBRARY} ${DYNAMICS_LIBRARY}
                        ${RBDL_LIBRARY} ${PRIMITIVES_LIBRARY})

add_executable(simviz_handcontroller src/simviz.cpp src/ParallelRobot.cpp)
target_link_libraries(simviz_handcontroller ${LINK_LIBRARIES})

add_executable(hc_controller src/ParallelRobot.cpp src/robot_controller.cpp)
target_link_libraries(hc_controller ${LINK_LIBRARIES})

add_executable(hc_visualizer src/ParallelRobot.cpp src/visualizer.cpp)
target_link_libraries(hc_visualizer ${LINK_LIBRARIES})

add_executable(hc_controller_imp src/impedance_controller.cpp src/ImpedanceControl.cpp src/ParallelRobot.cpp )
target_link_libraries(hc_controller_imp ${LINK_LIBRARIES})

# add_executable(Inverse_kinematics src/Inverse_kinematics.cpp)
# target_link_libraries(Inverse_kinematics ${LINK_LIBRARIES})


# add_executable(simviz_franka src/simviz_franka.cpp)
# target_link_libraries(simviz_franka ${LINK_LIBRARIES})
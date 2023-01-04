add_executable(robotTrajExperiments robotTraj.cpp ${CMAKE_SOURCE_DIR}src/Vicarious.cpp ${CMAKE_SOURCE_DIR}/src/VicariousRobot.cpp)

target_include_directories(robotTrajExperiments PRIVATE ${CMAKE_SOURCE_DIR}/lib/rigid-body-motion/include ${CMAKE_SOURCE_DIR}/include ${CMAKE_SOURCE_DIR}/lib/Trajectory/include)

target_link_libraries(robotTrajExperiments RBMotion Trajectory)

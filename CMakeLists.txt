CMAKE_MINIMUM_REQUIRED( VERSION 2.8 )
PROJECT( SLAM_SIM )

set(CMAKE_CXX_COMPILER "g++")
set(CMAKE_BUILD_TYPE Release )
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O2")
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(Eigen3 REQUIRED)
find_package(Pangolin 0.2 REQUIRED)

include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
    ${Pangolin_INCLUDE_DIRS}
)

set( INCLUDE_FILES
    ${PROJECT_SOURCE_DIR}/include/converter.h 
    ${PROJECT_SOURCE_DIR}/include/definition.h
    ${PROJECT_SOURCE_DIR}/include/key_frame.h
    ${PROJECT_SOURCE_DIR}/include/map.h
    ${PROJECT_SOURCE_DIR}/include/map_point.h
    ${PROJECT_SOURCE_DIR}/include/system_simulator.h
    ${PROJECT_SOURCE_DIR}/include/trajectory_parameter.h
    ${PROJECT_SOURCE_DIR}/include/viewer.h
)

set( SRC_FILES  
    ${PROJECT_SOURCE_DIR}/src/converter.cpp
    ${PROJECT_SOURCE_DIR}/src/key_frame.cpp
    ${PROJECT_SOURCE_DIR}/src/map.cpp
    ${PROJECT_SOURCE_DIR}/src/map_point.cpp
    ${PROJECT_SOURCE_DIR}/src/system_simulator.cpp
    ${PROJECT_SOURCE_DIR}/src/trajectory_parameter.cpp
    ${PROJECT_SOURCE_DIR}/src/viewer.cpp
)

add_executable( test_slam_sim example/main.cpp ${INCLUDE_FILES} ${SRC_FILES} )
target_link_libraries( test_slam_sim ${Pangolin_LIBRARIES} )




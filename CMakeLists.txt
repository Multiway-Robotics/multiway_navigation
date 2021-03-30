cmake_minimum_required(VERSION 2.8)
project(multiway_navigation)

set(CMAKE_BUILD_TYPE Release )
if( CMAKE_BUILD_TYPE STREQUAL "Release")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -std=c++14 -fPIC")
else()
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 -std=c++14 -fPIC")
endif()

file(GLOB_RECURSE HEADER_FILES include/*.h)
file(GLOB_RECURSE MAPPER_SRC_FILES src/multiway_navigation/mapper/*.cpp)
file(GLOB_RECURSE LOCALIZER_SRC_FILES src/multiway_navigation/localizer/*.cpp)

set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} /opt/ceres-solver)
find_package(Ceres REQUIRED)

if(${Ceres_FOUND})
  message(STATUS "Ceres include path: ${CERES_INCLUDE_DIRS}")
  message(STATUS "Ceres library path: ${CERES_LIBRARIES}")
endif()

find_package(catkin REQUIRED COMPONENTS
message_filters
laser_geometry
)

find_package(Boost REQUIRED COMPONENTS 
system 
serialization
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pointcloud_filters
  CATKIN_DEPENDS message_filters laser_geometry
)


include_directories(include/multiway_navigation ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS} ${sirius_log_INCLUDE_DIRS} )

set(LIB_SOURCE_FILES  src/multiway_navigation/base/landmark_map.cpp
                      src/multiway_navigation/base/land_mark.cpp
                      src/multiway_navigation/localizer/reflector_recognizer.cpp
                      src/multiway_navigation/localizer/landmark_localizer.cpp
                      src/multiway_navigation/mapper/landmark_mapper.cpp
                      src/multiway_navigation/filter/mean_filter.cpp
                      src/multiway_navigation/filter/laser_corrector.cpp
                      src/multiway_navigation/filter/kalman.cpp
                      src/multiway_navigation/filter/kalman_interface.cpp
                      src/multiway_navigation/visualizer/visualizer.cpp
                      src/multiway_navigation/solver/least_square_solver.cpp
                      src/multiway_navigation/ros_node/tf_bridge.cpp
                      src/multiway_navigation/ros_node/ros_interface.cpp)

set(MAPPER_SOURCE_FILES src/multiway_navigation/ros_node/ros_node_main.cpp)


add_library(reflector_base ${LIB_SOURCE_FILES} ${HEADER_FILES})
target_link_libraries(reflector_base ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${CERES_LIBRARIES})

add_executable(${PROJECT_NAME} ${MAPPER_SOURCE_FILES} ${HEADER_FILES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${CERES_LIBRARIES} reflector_base)

add_executable(map_trans_solve "src/multiway_navigation/ros_node/map_trans_solve.cpp" ${HEADER_FILES})
target_link_libraries(map_trans_solve reflector_base)

# install executables and/or libraries
install(TARGETS reflector_base ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install files
foreach(dir launch)
install(DIRECTORY ${dir}/
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

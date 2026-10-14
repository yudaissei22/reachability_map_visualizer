cmake_minimum_required(VERSION 3.0.2)
project(reachability_map_visualizer_ironman_upper)

add_compile_options(-std=c++17)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  reachability_map_visualizer
  choreonoid_viewer
  roslib
  UronMan_models
  )

find_package(choreonoid REQUIRED)

set(CHOREONOID_LIBRARIES_ABS)
foreach(lib ${CHOREONOID_LIBRARIES})
  find_library(${lib}_abs NAMES ${lib} PATHS ${CHOREONOID_LIBRARY_DIRS})
  set(CHOREONOID_LIBRARIES_ABS ${CHOREONOID_LIBRARIES_ABS} ${${lib}_abs})
endforeach(lib)

catkin_package(
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${CHOREONOID_INCLUDE_DIRS}
)

add_library(CnoidReachabilityMapVisualizerSamplePlugin SHARED
  src/CnoidReachabilityMapVisualizerSamplePlugin.cpp
  src/jaxon_hand.cpp
  src/visualize.cpp
  src/view_4uu.cpp
  src/solvability_bar.cpp
  )
target_link_libraries(CnoidReachabilityMapVisualizerSamplePlugin
  ${catkin_LIBRARIES}
  ${CHOREONOID_BODY_PLUGIN_LIBRARIES}
  )
set_target_properties(CnoidReachabilityMapVisualizerSamplePlugin PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CHOREONOID_PLUGIN_DIR})

add_executable(view_4uu
  src/view_4uu_main.cpp
  src/view_4uu.cpp
  )
target_link_libraries(view_4uu
  ${catkin_LIBRARIES}
  ${CHOREONOID_LIBRARIES_ABS}
  ${CHOREONOID_BODY_PLUGIN_LIBRARIES}
  )

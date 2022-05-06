cmake_minimum_required(VERSION 2.8.3)
cmake_policy(SET CMP0048 NEW)
project(motion_bench_maker) 
add_compile_options(-std=c++14)

set(LIBRARY_NAME ${PROJECT_NAME})

list(APPEND CATKIN_SOURCES
  robowflex_library
  robowflex_ompl
  pcl_conversions # to convert to pcl versions 
)

find_package(catkin REQUIRED COMPONENTS ${CATKIN_SOURCES})
#Octomap is not a a catkin package, a.k.a. it has to be found like this
find_package(octomap)
# needed to convert octomaps

SET(OpenGL_GL_PREFERENCE "GLVND")
find_package(gl_depth_sim)

find_package(Threads REQUIRED)

catkin_package(
    LIBRARIES ${LIBRARY_NAME}
    CATKIN_DEPENDS ${CATKIN_SOURCES}
    DEPENDS
    INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/include
    )

list(APPEND SOURCES
    src/setup.cpp
    src/scene_sampler.cpp
    src/yaml.cpp
    src/problem_generator.cpp
    src/octomap_generator.cpp
  )

list(APPEND INCLUDES
  ${CMAKE_CURRENT_LIST_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${gl_depth_sim_INCLUDE_DIRS}
  )

list(APPEND LIBRARIES
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  gl_depth_sim::gl_depth_sim
  gl_depth_sim::glad
  gl_depth_sim::gl_depth_sim_interfaces
  yaml-cpp
  )


include_directories(SYSTEM ${INCLUDES})
link_directories(${catkin_LIBRARY_DIRS})
add_library(${LIBRARY_NAME} ${SOURCES})
set_target_properties(${LIBRARY_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})
target_link_libraries(${LIBRARY_NAME} ${LIBRARIES})

add_executable(visualize scripts/visualize.cpp)
target_link_libraries(visualize ${LIBRARY_NAME})
add_dependencies(visualize ${catkin_EXPORTED_TARGETS})

add_executable(benchmark scripts/benchmark.cpp)
target_link_libraries(benchmark ${LIBRARY_NAME})
add_dependencies(benchmark ${catkin_EXPORTED_TARGETS})

add_executable(paper_example scripts/paper_example.cpp)
target_link_libraries(paper_example ${LIBRARY_NAME})
add_dependencies(paper_example ${catkin_EXPORTED_TARGETS})

add_executable(generate scripts/generate.cpp)
target_link_libraries(generate ${LIBRARY_NAME})
add_dependencies(generate ${catkin_EXPORTED_TARGETS})

add_executable(sample_scenes scripts/sample_scenes.cpp)
target_link_libraries(sample_scenes ${LIBRARY_NAME})
add_dependencies(sample_scenes ${catkin_EXPORTED_TARGETS})

add_executable(urdf_scenes scripts/urdf_scenes.cpp)
target_link_libraries(urdf_scenes ${LIBRARY_NAME})
add_dependencies(urdf_scenes ${catkin_EXPORTED_TARGETS})

add_executable(sensed_scenes scripts/sensed_scenes.cpp)
target_link_libraries(sensed_scenes ${LIBRARY_NAME})
add_dependencies(sensed_scenes ${catkin_EXPORTED_TARGETS})

#add_executable(benchmark_simple scripts/benchmark_simple.cpp)
#target_link_libraries(benchmark_simple ${LIBRARY_NAME})
#add_dependencies(benchmark_simple ${catkin_EXPORTED_TARGETS})

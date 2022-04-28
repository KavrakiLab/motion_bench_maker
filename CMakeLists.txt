cmake_minimum_required(VERSION 2.8.3)
project(motion_bench_maker) 
add_compile_options(-std=c++14)

set(LIBRARY_NAME ${PROJECT_NAME})

list(APPEND CATKIN_SOURCES
  robowflex_library
  robowflex_ompl
  tf # Used for ROS example
  tf_conversions # Used for ROS example
)

find_package(catkin REQUIRED COMPONENTS ${CATKIN_SOURCES})
#Octomap is not a a catkin package, a.k.a. it has to be found like this
find_package(octomap REQUIRED)

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
  )

list(APPEND INCLUDES
  ${CMAKE_CURRENT_LIST_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  )

list(APPEND LIBRARIES
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  yaml-cpp
  )


include_directories(SYSTEM ${INCLUDES})
link_directories(${catkin_LIBRARY_DIRS})
add_library(${LIBRARY_NAME} ${SOURCES})
set_target_properties(${LIBRARY_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})
target_link_libraries(${LIBRARY_NAME} ${LIBRARIES})

add_executable(generate scripts/generate.cpp)
target_link_libraries(generate ${LIBRARY_NAME})
add_dependencies(generate ${catkin_EXPORTED_TARGETS})

add_executable(benchmark scripts/benchmark.cpp)
target_link_libraries(benchmark ${LIBRARY_NAME})
add_dependencies(benchmark ${catkin_EXPORTED_TARGETS})

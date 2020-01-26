cmake_minimum_required(VERSION 2.8.3)
project(slp_doa)

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  tf
  cmake_modules
  state_lattice_planner
  dynamic_obstacle_avoidance_planner
)

find_package(Eigen3 REQUIRED COMPONENTS system)

find_package(OpenMP REQUIRED)
if (OPENMP_FOUND)
  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()


catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES slp_doa
#  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(slp_doa_lib
  src/slp_doa.cpp
)

add_dependencies(slp_doa_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(slp_doa src/slp_doa_node.cpp)
add_executable(fake_local_costmap_generator src/fake_local_costmap_generator.cpp)
add_executable(fake_local_goal_publisher src/fake_local_goal_publisher.cpp)

add_dependencies(slp_doa ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(fake_local_costmap_generator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(fake_local_goal_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(slp_doa_lib
  ${catkin_LIBRARIES}
)
target_link_libraries(slp_doa
  ${catkin_LIBRARIES}
  slp_doa_lib
)
target_link_libraries(fake_local_costmap_generator
  ${catkin_LIBRARIES}
)
target_link_libraries(fake_local_goal_publisher
  ${catkin_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_rostest_gtest(slp_doa_test
        test/slp_doa.test
        test/slp_doa_test.cpp
    )
    target_link_libraries(slp_doa_test
        ${catkin_LIBRARIES}
        slp_doa_lib
    )
endif()

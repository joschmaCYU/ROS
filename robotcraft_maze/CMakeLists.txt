cmake_minimum_required(VERSION 3.0.2)
project(robotcraft_maze)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  stage_ros
  std_msgs
  tf
)

find_package(OpenCV REQUIRED)

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(maze_BASICsolver src/maze_BASICsolver.cpp ${catkin_LIBRARIES})
target_link_libraries(maze_BASICsolver ${catkin_LIBRARIES})
add_dependencies(maze_BASICsolver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(maze_PROsolver src/maze_PROsolver.cpp)
target_include_directories(maze_PROsolver PRIVATE ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(maze_PROsolver PRIVATE ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(maze_PROsolver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
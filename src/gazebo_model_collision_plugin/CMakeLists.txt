cmake_minimum_required(VERSION 2.8.3)
project(gazebo_model_collision_plugin)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  gazebo_ros
  gazebo_plugins
)

find_package(gazebo REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")


add_message_files(DIRECTORY msgs
  FILES
  Contact.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs gazebo_ros gazebo_plugins
)

add_library(gazebo_collision_sensor SHARED src/gazebo_model_collision_plugin.cpp)
add_dependencies(gazebo_collision_sensor 
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
)
target_link_libraries(gazebo_collision_sensor ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

install(DIRECTORY
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS gazebo_collision_sensor
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

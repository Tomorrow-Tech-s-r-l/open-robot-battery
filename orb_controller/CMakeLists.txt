cmake_minimum_required(VERSION 3.0.2)
project(orb_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  std_srvs
)

add_message_files(
  FILES
  PowerStatus.msg
  PDStatus.msg
)

add_service_files(
  FILES
  SetVoltageCurrent.srv
  GetStatus.srv
  RequestPDRole.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs message_runtime std_srvs
  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/ezpd_bcr_i2c.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(orb_controller_node src/orb_controller_node.cpp)
target_link_libraries(orb_controller_node ${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(orb_controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS ${PROJECT_NAME} orb_controller_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
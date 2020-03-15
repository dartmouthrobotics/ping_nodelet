cmake_minimum_required(VERSION 2.8.3)
project(ping_nodelet)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslint
  nodelet
  pluginlib
  message_generation
)

add_message_files(
  FILES
  Ping.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ping_depth_nodelet
  CATKIN_DEPENDS roscpp
                 roslint
                 nodelet
                 pluginlib
                 message_generation
                 message_runtime
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_library(ping_depth_nodelet
  src/nodelet.cpp
)

add_dependencies(ping_depth_nodelet
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(ping_depth_nodelet
  ${catkin_LIBRARIES}
)

roslint_cpp(
  src/nodelet.cpp
  include/${PROJECT_NAME}/ping_depth.h
  include/${PROJECT_NAME}/ping-message-all.h
  include/${PROJECT_NAME}/ping-message-bluebps.h
  include/${PROJECT_NAME}/ping-message-common.h
  include/${PROJECT_NAME}/ping-message-ping1d.h
  include/${PROJECT_NAME}/ping-message-ping360.h
  include/${PROJECT_NAME}/ping-message.h
  include/${PROJECT_NAME}/ping-parser.h
)

install(TARGETS ping_depth_nodelet
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY plugins
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_pa1.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

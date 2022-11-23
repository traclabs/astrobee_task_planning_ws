cmake_minimum_required(VERSION 2.8.3)
project(astrobee_task_planning_examples)

set(DEPS
  roscpp
  ff_msgs
  behaviortree_cpp_v3
)

find_package(catkin REQUIRED
  COMPONENTS
  ${DEPS}
)

find_package(Boost REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES #lifecycle
  CATKIN_DEPENDS ${DEPS}
  DEPENDS 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(undock_bt_action SHARED src/plugins/actions/undock_bt_action.cpp)
list(APPEND plugin_libs undock_bt_action)


foreach(bt_plugin ${plugin_libs})
  target_link_libraries(${bt_plugin} ${catkin_LIBRARIES})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()


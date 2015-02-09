cmake_minimum_required(VERSION 2.4.6)
project(lcsr_nettools)

find_package(catkin REQUIRED COMPONENTS
  message_generation sensor_msgs roscpp message_filters tf pluginlib rviz
  topic_tools)

add_message_files(FILES
  HeaderSample.msg
  StatisticsMeasurements.msg
  TopicMeasurements.msg
  TopicStatistics.msg
  )

generate_messages(DEPENDENCIES std_msgs)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/statistics.cpp)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)


# TODO: update rviz plugin for hydro
#add_subdirectory(rviz_plugins)

add_executable(statistics_node src/statistics_node.cpp)
target_link_libraries(statistics_node lcsr_nettools ${catkin_LIBRARIES})


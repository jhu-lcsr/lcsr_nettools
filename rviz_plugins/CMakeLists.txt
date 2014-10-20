cmake_minimum_required(VERSION 2.4.6)

find_package(Boost REQUIRED COMPONENTS thread signals filesystem system program_options)
include_directories(${Boost_INCLUDE_DIRS})

## This plugin includes Qt widgets, so we must include Qt like so:
find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

## I prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)

qt4_wrap_cpp(MOC_FILES
  src/camera_display.h
)

set(SOURCE_FILES
  src/init.cpp
  src/camera_display.cpp
  ${MOC_FILES}
)

add_library(lcsr_nettools_rviz ${SOURCE_FILES})

target_link_libraries(lcsr_nettools_rviz
  ${PROJECT_NAME} ${catkin_LIBRARIES} ${QT_LIBRARIES} ${OGRE_LIBRARIES} ${Boost_LIBRARIES})

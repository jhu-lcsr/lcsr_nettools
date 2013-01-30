cmake_minimum_required(VERSION 2.4.6)

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

rosbuild_add_library(lcsr_nettools_rviz ${SOURCE_FILES})
                                    
target_link_libraries(lcsr_nettools_rviz ${PROJECT_NAME} ${QT_LIBRARIES} ${OGRE_LIBRARIES})
rosbuild_link_boost(lcsr_nettools_rviz thread signals filesystem system program_options)

cmake_minimum_required(VERSION 3.5)

project(kiwi_gazebo)
add_compile_options(-std=c++14)

find_package(ament_cmake REQUIRED)
find_package (Qt5Widgets REQUIRED)
find_package (Qt5Core REQUIRED)
find_package(Protobuf REQUIRED)
find_package(gazebo REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  models
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  worlds
  DESTINATION share/${PROJECT_NAME}/
)

include_directories(
  include ${catkin_INCLUDE_DIRS} lib
)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
  pkg_check_modules(OGRE OGRE)
endif()
pkg_check_modules(OGRE-Terrain OGRE-Terrain)
include_directories(${OGRE-Terrain_INCLUDE_DIRS})
include_directories(${GAZEBO_INCLUDE_DIRS} ${OGRE_INCLUDE_DIRS})
include_directories(${Qt5Widgets_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS} ${OGRE_LIBRARY_DIRS})




set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(tinyxml2 SHARED 
  lib/tinyxml2.cpp)

add_library(labelCreator SHARED lib/labelCreator.cpp)

add_library(polygon_parser SHARED ${headers_MOC}
  src/polygon_parser.cpp)
  
target_link_libraries(polygon_parser ${GAZEBO_LIBRARIES}
    ${Qt5Core_LIBRARIES}
    ${Qt5Widgets_LIBRARIES}
    ${PROTOBUF_LIBRARIES}
     labelCreator)

ament_target_dependencies(polygon_parser rclcpp std_msgs gazebo_ros) 




ament_package()

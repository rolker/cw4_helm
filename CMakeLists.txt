cmake_minimum_required(VERSION 2.8.3)
project(cw4_helm)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  asv_msgs
  asv_srvs
  geometry_msgs
  geographic_msgs
  nav_msgs
  project11
  project11_msgs
  roscpp
  sensor_msgs
  project11_msgs
)

catkin_package(
  CATKIN_DEPENDS asv_msgs asv_srvs geometry_msgs roscpp project11 nav_msgs sensor_msgs project11_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/cw4_helm_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
)

 install(TARGETS ${PROJECT_NAME}_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )


cmake_minimum_required(VERSION 3.0.2)
project(rico_rrt_plugin)

add_compile_options(-std=c++14)

set(rico_rrt_plugin_ROS_DEPS nav_core costmap_2d roscpp  nav_msgs pluginlib dynamic_reconfigure)

find_package(catkin REQUIRED COMPONENTS
        ${rico_rrt_plugin_ROS_DEPS})

find_package(Boost REQUIRED COMPONENTS thread)


# generate_dynamic_reconfigure_options(
#         cfg/rico_rrt_plugin.cfg
# )

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES rico_rrt_plugin
        CATKIN_DEPENDS ${rico_rrt_plugin_ROS_DEPS}
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_library(rico_rrt_plugin src/rico_rrt_plugin.cpp)

add_dependencies(rico_rrt_plugin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rico_rrt_plugin ${catkin_LIBRARIES})

install(TARGETS rico_rrt_plugin
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        )

install(FILES bgp_plugin.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )





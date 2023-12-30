cmake_minimum_required(VERSION 2.8.3)
project(tf_ros_cookbook)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  tf2
  tf2_ros
  tf2_geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(tf_cookbook src/tf_cookbook.cpp)
target_link_libraries(tf_cookbook ${catkin_LIBRARIES})

add_executable(tf2_cookbook src/tf2_cookbook.cpp)
target_link_libraries(tf2_cookbook ${catkin_LIBRARIES})

install(TARGETS
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

# if(CATKIN_ENABLE_TESTING)
#   find_package(rostest REQUIRED)
#   add_rostest_gtest(testing-test launch/gtest.test tests/tests_1.cpp)
#   target_link_libraries(testing-test ${catkin_LIBRARIES} testing)
# endif()
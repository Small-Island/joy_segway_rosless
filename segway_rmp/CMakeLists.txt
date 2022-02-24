cmake_minimum_required(VERSION 2.8.3)
project(segway_rmp)

set(serial_PATH "${PROJECT_SOURCE_DIR}/../serial_out")

message("PROJECT_SOURCE_DIR = ${PROJECT_SOURCE_DIR}")

configure_file(
    "${PROJECT_SOURCE_DIR}/serialPathConfig.h.in"
    "${PROJECT_BINARY_DIR}/serialPathConfig.h"
)

include_directories(
    ${PROJECT_BINARY_DIR}
)

# find_package(catkin REQUIRED
#   COMPONENTS
#     geometry_msgs
#     message_generation
#     nav_msgs
#     roscpp
#     # serial
#     std_msgs
#     tf
#     sensor_msgs
# )

find_package(Boost REQUIRED COMPONENTS system thread)
# set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../../libsegwayrmp)
# find_package(libsegwayrmp REQUIRED)
set(libsegwayrmp_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/../libsegwayrmp/include)

# add_message_files(
#   DIRECTORY msg
#   FILES SegwayStatus.msg SegwayStatusStamped.msg VelocityStatus.msg AccelCmd.msg jyja.msg
# )

# generate_messages(DEPENDENCIES std_msgs)

include_directories(
  ${Boost_INCLUDE_DIRS}
  ${libsegwayrmp_INCLUDE_DIRS}
  # ${catkin_INCLUDE_DIRS}
)


# catkin_package(
#   DEPENDS libsegwayrmp
#   CATKIN_DEPENDS
#     geometry_msgs
#     message_runtime
#     nav_msgs
#     roscpp
#     serial
#     std_msgs
#     tf
#     sensor_msgs
# )


add_executable(segway_rmp_node segway_rmp_node.cpp)
# add_executable(accel_cmd src/accel_cmd.cpp)

message("libsegwayrmp_LIBRARIES: ${libsegwayrmp_LIBRARIES}")

# add_dependencies(segway_rmp_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# set(libsegwayrmp_LIBRARIES ${PROJECT_SOURCE_DIR}/../../libsegwayrmp/lib/libftd2xx.a;${PROJECT_SOURCE_DIR}/../../libsegwayrmp/lib/libsegwayrmp.so)
list(APPEND libsegwayrmp_LIBRARIES
    ${PROJECT_SOURCE_DIR}/../libsegwayrmp/lib/libftd2xx.a
    ${PROJECT_SOURCE_DIR}/../libsegwayrmp/lib/libsegwayrmp.so
)

target_link_libraries(segway_rmp_node
  ${Boost_LIBRARIES}
  ${libsegwayrmp_LIBRARIES}
  # ${catkin_LIBRARIES}
)

# target_link_libraries(accel_cmd
#   # ${Boost_LIBRARIES}
#   # ${libsegwayrmp_LIBRARIES}
#   # ${catkin_LIBRARIES}
# )

# install(TARGETS segway_rmp_node
#    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
#
# install(TARGETS accel_cmd
#    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
#
# install(DIRECTORY launch
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
# )

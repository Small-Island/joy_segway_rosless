cmake_minimum_required(VERSION 2.8.3)
project(segway_rmp)

set(serial_PATH "${PROJECT_SOURCE_DIR}/../serial_out")

configure_file(
    "${PROJECT_SOURCE_DIR}/serialPathConfig.h.in"
    "${PROJECT_BINARY_DIR}/serialPathConfig.h"
)

include_directories(
    ${PROJECT_BINARY_DIR}
)

find_package(Boost REQUIRED COMPONENTS system thread)
set(libsegwayrmp_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/../libsegwayrmp/include)

include_directories(
  ${Boost_INCLUDE_DIRS}
  ${libsegwayrmp_INCLUDE_DIRS}
)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/..)

add_executable(main_segway_rmp main_segway_rmp.cpp)
# set_target_properties(
#     main_segway_rmp
#     PROPERTIES
#     RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/..
# )

add_executable(test_segway_rmp test_segway_rmp.cpp)
# set_target_properties(
#     test_segway_rmp
#     PROPERTIES
#     RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/..
# )

add_executable(file_segway_rmp file_segway_rmp.cpp)
# set_target_properties(
#     file_segway_rmp
#     PROPERTIES
#     RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/..
# )

add_executable(file_count_segway_rmp file_count_segway_rmp.cpp)
# set_target_properties(
#     file_count_segway_rmp
#     PROPERTIES
#     RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/..
# )

add_executable(main main.cpp)
# set_target_properties(
#     main
#     PROPERTIES
#     RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/..
# )

add_executable(delay delay.cpp)

add_executable(main_ras main_ras.cpp)

add_executable(tst tst.cpp)
# set_target_properties(
#     tst
#     PROPERTIES
#     RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/..
# )

list(APPEND libsegwayrmp_LIBRARIES
    ${PROJECT_SOURCE_DIR}/../libsegwayrmp/lib/libftd2xx.a
    ${PROJECT_SOURCE_DIR}/../libsegwayrmp/lib/libsegwayrmp.so
)

target_link_libraries(main_segway_rmp
  ${Boost_LIBRARIES}
  ${libsegwayrmp_LIBRARIES}
)

target_link_libraries(test_segway_rmp
  ${Boost_LIBRARIES}
  ${libsegwayrmp_LIBRARIES}
)

target_link_libraries(file_segway_rmp
  ${Boost_LIBRARIES}
  ${libsegwayrmp_LIBRARIES}
)

target_link_libraries(file_count_segway_rmp
  ${Boost_LIBRARIES}
  ${libsegwayrmp_LIBRARIES}
)

target_link_libraries(main
  ${Boost_LIBRARIES}
  ${libsegwayrmp_LIBRARIES}
)

target_link_libraries(main_ras
  ${Boost_LIBRARIES}
  ${libsegwayrmp_LIBRARIES}
)

target_link_libraries(delay
  ${Boost_LIBRARIES}
  ${libsegwayrmp_LIBRARIES}
)


target_link_libraries(tst
  ${Boost_LIBRARIES}
  ${libsegwayrmp_LIBRARIES}
)

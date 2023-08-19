cmake_minimum_required(VERSION 3.16)

project(simple-linux-serial-port-library)

add_definitions(-DAPI_EXPORTS)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "-pthread -O3")

# notes 1, setup the install base path for the simple serial port shared library, include files and the test application
#       2, setup the RPATH for the test applications in bin/ directory
#       3, choose the /usr/local/bit-parallel path once ready to deploy in production (or edit to suit your needs)
#
#set(BIT_PARALLEL_INSTALL_ROOT "/usr/local/bit-parallel")
set(BIT_PARALLEL_INSTALL_ROOT "${PROJECT_SOURCE_DIR}/bit-parallel")
set(CMAKE_INSTALL_RPATH "${BIT_PARALLEL_INSTALL_ROOT}/lib")

include_directories(${PROJECT_SOURCE_DIR}/include)
add_library(serial-port-bp-v1.0 SHARED ${PROJECT_SOURCE_DIR}/src/serial_port.cpp)
install(TARGETS serial-port-bp-v1.0 DESTINATION ${BIT_PARALLEL_INSTALL_ROOT}/lib)

add_executable(test-serial-port ${PROJECT_SOURCE_DIR}/src/applications/test_serial_port.cpp)
target_link_libraries(test-serial-port serial-port-bp-v1.0)

# note, read_listener.hpp is used by other projects, so added to the root of the communications include directory
#
install(TARGETS test-serial-port DESTINATION ${BIT_PARALLEL_INSTALL_ROOT}/bin/communications/serial)
install(FILES ${PROJECT_SOURCE_DIR}/include/read_listener.hpp DESTINATION ${BIT_PARALLEL_INSTALL_ROOT}/include/communications)
install(FILES ${PROJECT_SOURCE_DIR}/include/serial_port.hpp DESTINATION ${BIT_PARALLEL_INSTALL_ROOT}/include/communications/serial)

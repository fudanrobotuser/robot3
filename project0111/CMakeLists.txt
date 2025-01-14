cmake_minimum_required(VERSION 3.5)
project(Ethercat_sdk)

set (CMAKE_CXX_STANDARD 11)
# check arch and os
message("-- CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64.*")
    set(ARCH amd64)
endif()
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "aarch64.*")
    set(ARCH arm64)
endif()

link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib/cpp/${ARCH})

option(USE_LIBRARY "Compile sources into a library" ON)

# third party include
include(CTest)
include(CPack)
# 包含头文件目录
include_directories(/usr/local/include/)
include_directories(./)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build)
# 添加对gdb的支持
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -w -g2 -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
set(SRCS  test_subscriber.cpp ecat_data_buffer.c)
add_executable(motor ${SRCS} )


if(USE_LIBRARY)
  target_link_libraries(motor pthread "${PROJECT_SOURCE_DIR}/lib/cpp/${ARCH}/libecat.a")
else()
  include_directories(ethercat/)
  add_subdirectory(ethercat/)
  target_link_libraries(motor pthread ecatStatic)
#  target_link_libraries(motor pthread ecatStatic)
endif()





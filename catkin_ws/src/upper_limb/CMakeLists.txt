cmake_minimum_required(VERSION 3.0.2)
project(upper_limb)

find_package(Boost REQUIRED COMPONENTS date_time thread)
find_package(catkin REQUIRED COMPONENTS message_generation rostime roscpp rosconsole roscpp_serialization sensor_msgs)

include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

include_directories(/usr/local/etherlab/include)
link_directories(/usr/local/etherlab/lib)
link_libraries(ethercat)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

macro(rostutorial T)
  add_executable(${T} src/${T}.cpp)
  target_link_libraries(${T} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
  install(TARGETS ${T}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endmacro()


foreach(dir
    upper_limb
    upper_limb2
    )
  rostutorial(${dir})
endforeach()


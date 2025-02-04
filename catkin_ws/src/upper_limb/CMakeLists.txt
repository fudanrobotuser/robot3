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

add_library(joint_data src/joint_data.cpp)
add_library(ethercat_interface_r src/ethercat_interface_r.cpp)

add_executable(upper_limb_r src/upper_limb_r.cpp)
target_link_libraries(upper_limb_r ${catkin_LIBRARIES} ${Boost_LIBRARIES} joint_data ethercat_interface_r)
install(TARGETS upper_limb_r
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


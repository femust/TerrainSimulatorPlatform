cmake_minimum_required(VERSION 2.8.3)
project(gazebo)

find_package(gazebo REQUIRED)

include_directories(
    ${GAZEBO_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}
    )

link_directories(
    ${GAZEBO_LIBRARY_DIRS}
    )

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(model_controller SHARED model_controller.cpp)
target_link_libraries(model_controller ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})





cmake_minimum_required(VERSION 3.5.1)
project(template_project)

set(CMAKE_CXX_STANDARD 14)

find_package(EEROS REQUIRED)
include_directories(${EEROS_INCLUDE_DIR})
link_directories(${EEROS_LIB_DIR})

add_executable(template_project main.cpp ControlSystem.cpp MyRobotSafetyProperties.cpp)
target_link_libraries(template_project eeros ucl ${CMAKE_DL_LIBS})

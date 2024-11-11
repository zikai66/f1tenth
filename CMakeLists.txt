project(f1)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(sources src/gap_follow/src/pid.cpp src/gap_follow/src/kalman.cpp src/gap_follow/src/kalman.cpp src/gap_follow/src/helper.hpp)


if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.11.0/lib)
target_link_libraries(pid /usr/local/lib/libuWS.so uv)



endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 


add_executable(pid ${sources})

target_link_libraries(pid z ssl uv uWS)
find_package(Eigen3 REQUIRED)
target_link_libraries(pid Eigen3::Eigen)


cmake_minimum_required(VERSION 3.5.0)
project(Serial VERSION 0.1.0 LANGUAGES C CXX)

find_package(Boost REQUIRED COMPONENTS system thread)

add_executable(Serial main.cpp)
target_link_libraries(Serial Boost::system Boost::thread)

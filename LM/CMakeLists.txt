cmake_minimum_required(VERSION 3.16)
project(LM)

set(CMAKE_CXX_STANDARD 14)

include_directories("F:/CLionProject/Vio/eigen3")

set(LM_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR} CACHE STRING "LM source location" FORCE)

include_directories("${LM_SOURCE_DIR}/include")

add_subdirectory(src)

add_executable(LM main.cpp)
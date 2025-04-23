cmake_minimum_required(VERSION 3.15)
project(Lab08)

set(CMAKE_CXX_STANDARD 11)

# Add Windows socket library
if(WIN32)
    link_libraries(wsock32)
endif()

# Add warning disables for MSVC
if(MSVC)
    add_compile_options(/wd4996)
else()
    add_compile_options(-Wno-deprecated)
endif()

add_executable(Lab08 scara_main.cpp scara.cpp robot.cpp)

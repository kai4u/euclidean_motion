cmake_minimum_required(VERSION 3.1)

project(euclidean_motion)

set(CMAKE_CXX_COMPILER /usr/bin/clang++)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)

# set(CMAKE_CXX_VISIBILITY_PRESET hidden)
# set(CMAKE_VISIBILITY_INLINES_HIDDEN ON)

add_library(EMotion STATIC src/motion.cpp src/motion.h src/utils.cpp src/utils.h)

target_include_directories(EMotion
    PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>")

add_executable(main src/main.cpp)

target_link_libraries(main EMotion)

add_subdirectory(test)

cmake_minimum_required(VERSION 3.4.1)

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -O3 -std=c++17 -Wall -Wextra -pedantic -DNDEBUG")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -std=c++17 -Wall -Wextra -pedantic -DNDEBUG")

add_library(
        gear_maniacs
        SHARED

        src/main/cpp/Main.cpp
        src/main/cpp/Cache.cpp
        src/main/cpp/robot/Encoders.cpp
        src/main/cpp/robot/EncodersDelegate.cpp)

find_library( # Sets the name of the path variable.
        log-lib

        # Specifies the name of the NDK library that
        # you want CMake to locate.
        log)

target_link_libraries( # Specifies the target library.
        gear_maniacs

        # Links the target library to the log library
        # included in the NDK.
        ${log-lib})
cmake_minimum_required(VERSION 3.16)

# Stop Ninja from auto-rerunning CMake.
# We will rerun configure manually when needed.
set(CMAKE_SUPPRESS_REGENERATION ON)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(klann-esp32)

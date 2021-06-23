cmake_minimum_required(VERSION 3.0.0)
project(sim-motion VERSION 0.1.0)

include(CTest)
enable_testing()

file(GLOB_RECURSE SOURCES src/*.cpp)

include_directories("${CMAKE_SOURCE_DIR}/SDL2-vc/include")
link_directories("${CMAKE_SOURCE_DIR}/SDL2-vc/lib/x64")

add_executable(sim-motion ${SOURCES})
target_link_libraries(sim-motion SDL2 SDL2main)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)

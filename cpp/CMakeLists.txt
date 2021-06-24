cmake_minimum_required(VERSION 3.0.0)
project(sim-motion VERSION 0.1.0)

set(CMAKE_BUILD_TYPE "Release")

if(MSVC)
  add_compile_options(/W3)
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
else()
  add_compile_options(-Wall -Wextra)
endif()

include(CTest)
enable_testing()

file(GLOB_RECURSE SOURCES src/*.cpp)

include_directories("${CMAKE_SOURCE_DIR}/SFML-vc/include")
link_directories("${CMAKE_SOURCE_DIR}/SFML-vc/lib")

add_executable(sim-motion ${SOURCES})
target_link_libraries(sim-motion sfml-audio-d sfml-graphics-d sfml-system-d sfml-window-d)

add_custom_command(TARGET sim-motion POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different
  "${PROJECT_SOURCE_DIR}/arial.ttf"
  $<TARGET_FILE_DIR:sim-motion>)

add_custom_command(TARGET sim-motion POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_directory
  "${PROJECT_SOURCE_DIR}/SFML-vc/bin"
  $<TARGET_FILE_DIR:sim-motion>)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)

cmake_minimum_required(VERSION 3.0.0)
project(sim-motion VERSION 0.1.0)

if(MSVC)
  add_compile_options(/W3)
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
else()
  add_compile_options(-Wall -Wextra)
endif()

include(CTest)
enable_testing()

file(GLOB_RECURSE SOURCES src/*.cpp)

if(MSVC)
  include_directories("${CMAKE_SOURCE_DIR}/libraries/SFML-vc/include")
  link_directories("${CMAKE_SOURCE_DIR}/libraries/SFML-vc/lib")
else()
  include_directories("${CMAKE_SOURCE_DIR}/libraries/SFML-gcc/include")
  link_directories("${CMAKE_SOURCE_DIR}/libraries/SFML-gcc/lib")
endif()

add_executable(sim-motion ${SOURCES})

if(MSVC)
  target_link_libraries(sim-motion sfml-audio-d sfml-graphics-d sfml-system-d sfml-window-d)
else()
  target_link_libraries(sim-motion sfml-audio sfml-graphics sfml-system sfml-window)
endif()

add_custom_command(TARGET sim-motion POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different
  "${PROJECT_SOURCE_DIR}/arial.ttf"
  $<TARGET_FILE_DIR:sim-motion>)

if(MSVC)
  add_custom_command(TARGET sim-motion POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_directory
    "${PROJECT_SOURCE_DIR}/libraries/SFML-vc/bin"
    $<TARGET_FILE_DIR:sim-motion>)
endif()

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)

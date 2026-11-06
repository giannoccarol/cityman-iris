cmake_minimum_required(VERSION 3.16)
project(citymaniris_engine LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(IRIS_WARNINGS -Wall -Wextra -Wshadow -Wconversion -Wno-sign-conversion)

set(IRIS_SOURCES
  src/image.cpp
  src/segment.cpp
  src/encode.cpp
  src/iris_api.cpp)

add_library(citymaniris_engine_static STATIC ${IRIS_SOURCES})
target_include_directories(citymaniris_engine_static PUBLIC include PRIVATE src)
target_compile_options(citymaniris_engine_static PRIVATE ${IRIS_WARNINGS})
set_target_properties(citymaniris_engine_static PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(citymaniris_engine SHARED ${IRIS_SOURCES})
target_include_directories(citymaniris_engine PUBLIC include PRIVATE src)
target_compile_options(citymaniris_engine PRIVATE ${IRIS_WARNINGS} -fvisibility=hidden)

add_executable(iris_unit_tests tests/unit_tests.cpp)
target_include_directories(iris_unit_tests PRIVATE src)
target_compile_options(iris_unit_tests PRIVATE ${IRIS_WARNINGS})
target_link_libraries(iris_unit_tests PRIVATE citymaniris_engine_static)
find_package(Threads REQUIRED)
target_link_libraries(iris_unit_tests PRIVATE Threads::Threads)

add_executable(iris_cli tools/iris_cli.cpp)
target_include_directories(iris_cli PRIVATE src)
target_compile_options(iris_cli PRIVATE ${IRIS_WARNINGS})
target_link_libraries(iris_cli PRIVATE citymaniris_engine_static)

enable_testing()
add_test(NAME unit COMMAND iris_unit_tests)

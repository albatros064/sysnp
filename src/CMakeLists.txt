cmake_minimum_required(VERSION 3.15)

project(sysnp VERSION 0.9)

set(CMAKE_CPP_FLAGS "-g")

set(THREADS_PREFER_PTHREAD_FLAG ON)
set(CMAKE_CXX_STANDARD 20)

find_package(Threads REQUIRED)
find_package(Boost COMPONENTS headers REQUIRED)

add_subdirectory(rapidyaml)

add_library(libsysnp OBJECT)
add_subdirectory(machine)
target_include_directories(libsysnp PRIVATE rapidyaml/src rapidyaml/ext/c4core/src)

add_executable(sysnp main.cpp $<TARGET_OBJECTS:libsysnp>)
target_link_libraries(sysnp PUBLIC Boost::headers ryml::ryml PRIVATE Threads::Threads)
target_include_directories(sysnp PRIVATE rapidyaml/src rapidyaml/ext/c4core/src)


if (NOT CMAKE_BUILD_TYPE MATCHES Release)
    find_package(Boost COMPONENTS unit_test_framework REQUIRED)
    add_executable(test $<TARGET_OBJECTS:libsysnp>)
    target_link_libraries(test PRIVATE Threads::Threads Boost::unit_test_framework ryml::ryml)
    target_include_directories(test PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/machine> PRIVATE rapidyaml/src rapidyaml/ext/c4core/src)
    add_subdirectory(tests)
endif()

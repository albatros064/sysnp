cmake_minimum_required(VERSION 3.15)

project(sysnp VERSION 0.9)

set(CMAKE_CPP_FLAGS "-g")

set(THREADS_PREFER_PTHREAD_FLAG ON)
set(CMAKE_CXX_STANDARD 20)

find_package(Threads REQUIRED)
add_executable(sysnp main.cpp)
target_link_libraries(sysnp PUBLIC config++ PRIVATE Threads::Threads)
find_package(Boost COMPONENTS unit_test_framework REQUIRED)
add_executable(test)
target_link_libraries(test PUBLIC config++ PRIVATE Threads::Threads Boost::unit_test_framework)

add_subdirectory(machine)
add_subdirectory(tests)

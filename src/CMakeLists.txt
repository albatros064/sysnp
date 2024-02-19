cmake_minimum_required(VERSION 3.15)

project(sysnp VERSION 0.1)

set(CMAKE_CPP_FLAGS "-g")

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

#find_package(PkgConfig)
#pkg_check_modules(LIBCONFIG_PLUSPLUS libconfig++ REQUIRED)

add_executable(sysnp main.cpp machine.cpp machine.h device.h)
target_link_libraries(sysnp PUBLIC config++ PRIVATE Threads::Threads)
#target_link_libraries(sysnp PRIVATE Threads::Threads)
add_subdirectory(nbus)

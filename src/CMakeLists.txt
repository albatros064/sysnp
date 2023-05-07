cmake_minimum_required(VERSION 3.15)

project(sysnp VERSION 0.1)

set(CMAKE_CPP_FLAGS "-g")

#find_package(PkgConfig)
#pkg_check_modules(LIBCONFIG_PLUSPLUS libconfig++ REQUIRED)

add_executable(sysnp main.cpp machine.cpp machine.h device.h)
target_link_libraries(sysnp config++)
add_subdirectory(nbus)

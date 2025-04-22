target_sources(test
    PRIVATE
        test_main.cpp
        nbus.cpp
        memory.cpp
        serial.cpp
)
add_subdirectory(cpu)

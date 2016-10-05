#include <iostream>

#include "machine.h"

int main(int argc, char* argv[]) {
    sysnp::Machine machine;
    machine.load("hardware.conf");

    machine.run();
    
    return 0;
}


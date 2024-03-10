#include <iostream>

#include "machine/machine.h"

int main(int argc, char* argv[]) {
    std::shared_ptr<sysnp::Machine> machine = std::make_shared<sysnp::Machine>();
    machine->load("hardware.conf");

    machine->run();
    
    return 0;
}


#include <iostream>

#include "memory.h"

namespace sysnp {

Memory::Memory() {
}

void Memory::init(Machine &machine, const libconfig::Setting &setting) {
    std::cout << "Memory::init()" << std::endl;
}
void Memory::postInit(Machine &machine) {
    std::cout << "Memory::postInit()" << std::endl;
}
void Memory::clock(NBus &bus) {
    std::cout << "Memory::clock()" << std::endl;
}


}; // namespace

#include <iostream>
#include <fstream>
#include <array>

#include "memory.h"

namespace sysnp {

Memory::Memory():capacity(0),deviceAddress(0x1f0000),biosEnabled(true),data(0),biosData(0) {
}

Memory::~Memory() {
    if (data != 0) {
        delete data;
    }
    if (biosData != 0) {
        delete biosData;
    }
    data = 0;
    biosData = 0;
}

void Memory::init(Machine *machine, const libconfig::Setting &setting) {
    this->machine = machine;
    const libconfig::Setting &modules = setting["modules"];
    int moduleCount = modules.getLength();
    for (int i = 0; i < moduleCount; i++) {
        int moduleSize = 0;
        modules[i].lookupValue("capacity", moduleSize);
        capacity += moduleSize;
    }
    setting.lookupValue("device", deviceAddress);

    setting.lookupValue("file", biosFile);
    setting.lookupValue("bios", biosAddress);

    machine->debug("Memory::init()");
    machine->debug(" Found " + std::to_string(moduleCount) + " modules for " + std::to_string(capacity) + "KB");
    machine->debug(" Found bios file \"" + biosFile + "\", located at " + std::to_string(biosAddress));
}
void Memory::postInit() {
    machine->debug("Memory::postInit()");

    data = new uint8_t[capacity * 1024];
    if (data != 0) {
        machine->debug(" Allocated " + std::to_string(capacity) + "KB RAM");
    }

    biosData = new uint8_t[0x10000];
    if (biosData != 0) {
        machine->debug(" Allocated 64KB ROM");
        // Load bios file into ROM
        machine->readFile(biosFile, biosData, 0x10000);
        machine->debug("Loaded bios ROM from \"" + biosFile + "\"");
    }
}
void Memory::clock(NBus &bus) {
    std::cout << "Memory::clock()" << std::endl;

    bool read = bus.busRead();
    bool write = bus.busWrite();

    if (read || write) {
        uint32_t address = bus.busAddress();
        uint16_t data = bus.busData();
    }
}


}; // namespace

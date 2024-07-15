#include <iostream>
#include <iomanip>
#include <fstream>
#include <array>

#include "memory.h"

namespace sysnp {

namespace nbus {

void Memory::init(const libconfig::Setting &setting) {
    const libconfig::Setting &moduleConfig = setting["modules"];
    int moduleCount = moduleConfig.getLength();
    uint32_t    start;
    uint32_t    size;
    uint32_t readLatency;
    uint32_t writeLatency;
    bool        rom;
    std::string name;
    std::string file;

    setting.lookupValue("ioHole", ioHoleAddress);
    setting.lookupValue("ioHoleSize", ioHoleSize);

    uint32_t capacity = 0;
    for (int i = 0; i < moduleCount; i++) {
        const libconfig::Setting &module = moduleConfig[i];
        module.lookupValue("start",         start);
        module.lookupValue("name",          name);
        module.lookupValue("size",          size);
        module.lookupValue("rom",           rom);
        module.lookupValue("file",          file);
        module.lookupValue("readLatency",   readLatency);
        module.lookupValue("writeLatency",  writeLatency);

        modules.push_back(std::make_shared<MemoryModule>(start, size * 1024, rom, file, readLatency, writeLatency, name));

        capacity += size;
    }

    machine->debug("Memory::init()");
    machine->debug(" Found " + std::to_string(moduleCount) + " modules for " + std::to_string(capacity) + "KB");
}

void Memory::postInit() {
    phase = BusPhase::BusIdle;
    holdup = 0;
}

void Memory::clockUp() {
    machine->debug("Memory::clockUp()");

    switch (phase) {
        case BusPhase::BusActive:
            if (selectedModule && !writeLatch) {
                dataLatch = selectedModule->read(addressLatch);
                dataLatch |= selectedModule->read(addressLatch + 1) << 8;
                interface->assertSignal(NBusSignal::Data, dataLatch);
            }
            break;
        default:
            interface->deassertSignal(NBusSignal::Data);
            break;
    }
}

void Memory::clockDown() {
    machine->debug("Memory::clockDown()");

    uint32_t read    = interface->senseSignal(NBusSignal::ReadEnable);
    uint32_t write   = interface->senseSignal(NBusSignal::WriteEnable);
    uint32_t address = interface->senseSignal(NBusSignal::Address);
    uint32_t data    = interface->senseSignal(NBusSignal::Data);

    switch (phase) {
        case BusPhase::BusWait:
            if (holdup <= 0) {
                phase = BusPhase::BusActive;
                addressLatch = address;
                readLatch = read;
                writeLatch = write;
                for (auto module: modules) {
                    if (module->containsAddress(addressLatch)) {
                        selectedModule = module;
                        break;
                    }
                }
            }
            else {
                holdup--;
            }
            break;
        case BusPhase::BusActive:
            for (auto module: modules) {
                if (module->containsAddress(addressLatch)) {
                    selectedModule = module;
                    if (writeLatch & 1) {
                        selectedModule->write(addressLatch, data & 0xff);
                    }
                    if (writeLatch & 2) {
                        selectedModule->write(addressLatch + 1, data >> 8);
                    }
                    break;
                }
            }
            addressLatch = address;
            if (!read) {
                phase = BusPhase::BusCleanup;
            }
            break;
        case BusPhase::BusBegin:
            if (!read) {
                phase = BusPhase::BusCleanup;
            }
            break;
        case BusPhase::BusCleanup:
            phase = BusPhase::BusIdle;
            break;
        case BusPhase::BusIdle:
        default:
            if (read) {
                if (address < ioHoleAddress || address >= (ioHoleAddress + ioHoleSize)) {
                    // We are selected
                    phase = BusPhase::BusWait;
                }
                else {
                    // We are not selected
                    phase = BusPhase::BusBegin;
                }
            }
            break;
    }
}

std::string Memory::command(std::stringstream &input)  {
    std::stringstream response;
    std::string locationStr;
    uint16_t length = 64;
    uint32_t location;

    input >> locationStr >> length;

    location = std::stoul(locationStr, nullptr, 0);
    location >>= 3;
    location <<= 3;
    for (int i = 0; i < length / 8; i++) {
        response << std::setw(8) << std::setfill('0') << std::hex << (location + (i << 3)) << " ";
        for (int b = 0; b < 8; b++) {
            uint32_t address = location + (i << 3) + b;
            for (auto module: modules) {
                if (module->containsAddress(address)) {
                    uint8_t datum = module->read(address);
                    response << " " << std::setw(2) << std::setfill('0') << std::hex << (int) datum;
                    break;
                }
            }
        }
        response << std::endl;
    }
    response << "Ok.";
    return response.str();
}

MemoryModule::MemoryModule(uint32_t start, uint32_t size, bool rom, std::string romFile, uint8_t readLatency, uint8_t writeLatency, std::string name):
        startAddress(start), size(size), rom(rom), readLatency(readLatency), writeLatency(writeLatency), name(name) {
    data = new uint8_t[size];
    if (rom) {
        std::ifstream fin(romFile, std::ifstream::binary);
        fin.read((char *) data, size);
    }
}
MemoryModule::~MemoryModule() {
    if (data != 0) {
        delete data;
        data = 0;
    }
}

bool MemoryModule::containsAddress(uint32_t address) {
    return address >= startAddress && address < (startAddress + size);
}
std::string MemoryModule::getName() {
    return name;
}
uint8_t MemoryModule::getReadLatency() {
    return readLatency;
}
uint8_t MemoryModule::getWriteLatency() {
    return writeLatency;
}

uint8_t MemoryModule::read(uint32_t address) {
    if (address < startAddress || address >= (startAddress + size)) {
        return 0;
    }

    return data[address - startAddress];
}
void MemoryModule::write(uint32_t address, uint8_t datum) {
    if (rom || address < startAddress || address >= (startAddress + size)) {
        return;
    }
    data[address - startAddress] = datum;
}

}; // namespace nbus

}; // namespace sysnp

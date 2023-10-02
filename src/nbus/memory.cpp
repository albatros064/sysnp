#include <iostream>
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

void Memory::postInit() {}

void Memory::clockUp() {
    machine->debug("Memory::clockUp()");
    if (status != MemoryStatus::Ready && status != MemoryStatus::Cleanup) {
        if (latency > 0) {
            interface->assert(NBusSignal::NotReady, 1);
            --latency;
        }
        else {
            interface->deassert(NBusSignal::NotReady);
            if (status == MemoryStatus::ReadLatency) {
                uint16_t data = selectedModule->read(addressLatch);
                data |= selectedModule->read(addressLatch + 1) << 8;
                interface->assert(NBusSignal::Data, data);
            }
            else if (status == MemoryStatus::WriteLatency) {
                if (writeLatch & 1) {
                    selectedModule->write(addressLatch, dataLatch & 0xff);
                }
                if (writeLatch & 2) {
                    selectedModule->write(addressLatch + 1, (dataLatch >> 8) & 0xff);
                }
            }
            status = MemoryStatus::Cleanup;
        }
    }
    else if (status == MemoryStatus::Cleanup) {
        interface->deassert(NBusSignal::Data);
        interface->deassert(NBusSignal::NotReady);

        status = MemoryStatus::Ready;
    }
}

void Memory::clockDown() {
    machine->debug("Memory::clockDown()");

    uint32_t read  = interface->sense(NBusSignal::ReadEnable);
    uint32_t write = interface->sense(NBusSignal::WriteEnable);

    // Are we in a position to respond to bus activity?
    if (status == MemoryStatus::Ready) {
        machine->debug("Latching bus lines");
        if (read || write) {
            machine->debug("Read or write requested");
            addressLatch = interface->sense(NBusSignal::Address) & 0xffffffffe;
            dataLatch    = interface->sense(NBusSignal::Data   );
            readLatch = read;
            writeLatch = write;

            if (addressLatch < ioHoleAddress || addressLatch >= (ioHoleAddress + ioHoleSize)) {
                machine->debug("Memory - not in io hole");
                for (auto module: modules) {
                    if (module->containsAddress(addressLatch)) {
                        machine->debug("Memory - handled by module " + module->getName());
                        selectedModule = module;
                        latency = 0;
                        if (read) {
                            status = MemoryStatus::ReadLatency;
                            latency = module->getReadLatency();
                        }
                        else if (write) {
                            status = MemoryStatus::WriteLatency;
                            latency = module->getWriteLatency();
                        }
                        else {
                            latency = 0;
                        }
                        break;
                    }
                }
            }
            else {
                machine->debug("Memory - address in io hole. Skipping");
            }
        }
    }
}

std::string Memory::command(std::stringstream &input)  {
    return "Ok.";
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

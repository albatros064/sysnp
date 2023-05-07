#include <iostream>
#include <fstream>
#include <array>

#include "memory.h"
#include "util.h"

namespace sysnp {

Memory::Memory():status(MemoryStatus::Ready) {}

Memory::~Memory() {}

void Memory::init(std::shared_ptr<Machine> machine, const libconfig::Setting &setting) {
    this->machine = machine;

    const libconfig::Setting &moduleConfig = setting["modules"];
    int moduleCount = moduleConfig.getLength();
    uint32_t    start;
    uint32_t    size;
    uint32_t readLatency;
    uint32_t writeLatency;
    bool        rom;
    std::string file;

    setting.lookupValue("ioHole", ioHoleAddress);
    setting.lookupValue("ioHoleSize", ioHoleSize);

    uint32_t capacity = 0;
    for (int i = 0; i < moduleCount; i++) {
        const libconfig::Setting &module = moduleConfig[i];
        module.lookupValue("start",         start);
        module.lookupValue("size",          size);
        module.lookupValue("rom",           rom);
        module.lookupValue("file",          file);
        module.lookupValue("readLatency",   readLatency);
        module.lookupValue("writeLatency",  writeLatency);

        modules.push_back(std::make_shared<MemoryModule>(start, size * 1024, rom, file, readLatency, writeLatency));

        capacity += size;
    }

    machine->debug("Memory::init()");
    machine->debug(" Found " + std::to_string(moduleCount) + " modules for " + std::to_string(capacity) + "KB");
}

void Memory::postInit() {}

void Memory::clockUp() {
    machine->debug("Memory::clockUp()");

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
                for (auto module: modules) {
                    if (module->containsAddress(addressLatch)) {
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
        }
    }
}

void Memory::clockDown() {
    machine->debug("Memory::clockDown()");
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

std::string Memory::output(uint8_t mode) {
    return "";
}

MemoryModule::MemoryModule(uint32_t start, uint32_t size, bool rom, std::string romFile, uint8_t readLatency, uint8_t writeLatency):
        startAddress(start), size(size), rom(rom), readLatency(readLatency), writeLatency(writeLatency) {
    data = new uint8_t[size];
    if (rom) {
        
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

}; // namespace

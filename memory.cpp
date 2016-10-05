#include <iostream>
#include <fstream>
#include <array>

#include "memory.h"
#include "util.h"

namespace sysnp {

Memory::Memory():capacity(0),deviceAddress(0x1f0000),biosEnabled(true),data(0),biosData(0),status(-1) {
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
    machine->debug(" Found bios file \"" + biosFile + "\", located at " + sysnp::to_hex(biosAddress, true));
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
    machine->debug("Memory::clock()");

    bool read = bus.busRead();
    bool write = bus.busWrite();

    // Are we in a position to respond to bus activity?
    if (status < 0) {
        machine->debug("Latching bus lines");
        if (read || write) {
            machine->debug("Read or write requested");
            addressLatch = bus.busAddress();
            dataLatch    = bus.busData();
            status = -1;

            if (addressLatch < 0x1f0000 || addressLatch >= 0x400000) {
                // this is a general memory read/write
                if (biosEnabled && addressLatch >= biosAddress && addressLatch < biosAddress + 0x10000) {
                    // this is a read from the bios rom
                    addressLatch -= biosAddress;
                    status = 1;
                }
                status += 10;
                if (write) {
                    status++;
                }
            }
            else if ((addressLatch & ~0xf) == deviceAddress) {
                // this is a command directed at us
                deviceCommand(bus);
            }
        }
    }
    else {
        if (status > 5) {
            machine->debug("Latency cycle");
            status -= 3;
            // are we trying to write to bios rom?
            if (status == 9) {
                // we've waited a cycle. ready for the next command
                status = -1;
            }
        }
        else if (status == 4) {
            machine->debug("Write cycle");
            data[addressLatch] = dataLatch;
            status = -1;
        }
        else {
            machine->debug("Burst cycle");
            if (dataLatch < 1) {
                status = -1;
            }
            else {
                uint16_t *d = (uint16_t*) data;
                if (status == 5) {
                    d = (uint16_t*) biosData;
                }
                // burst data onto the bus
                bus.busAddress(true, addressLatch);
                bus.busData   (true, d[addressLatch]);
                addressLatch += 2;
                dataLatch    -= 1;
            }
        }
    }
}

void Memory::deviceCommand(NBus &bus) {
}


}; // namespace

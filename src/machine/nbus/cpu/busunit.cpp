#include "n16r.h"
#include <iostream>

namespace sysnp {

namespace nbus {

namespace n16r {

void BusUnit::setBusInterface(std::shared_ptr<NBusInterface> interface) {
    this->interface = interface;
}
void BusUnit::reset() {
    phase = BusPhase::Idle;
    readReady = false;
    readPriority = false;
    hasLowPriority = false;
    hasHighPriority = false;
    hasHighPriorityWrite = false;
    interruptState = 0;
}

void BusUnit::clockUp() {
    interface->deassert(NBusSignal::Address);
    interface->deassert(NBusSignal::Data);
    interface->deassert(NBusSignal::WriteEnable);
    interface->deassert(NBusSignal::ReadEnable);

    if (phase == BusPhase::Idle) {
        if (hasHighPriority) {
            interface->assert(NBusSignal::Address, highPriorityAddress);
            if (hasHighPriorityWrite) {
                interface->assert(NBusSignal::Data, highPriorityData);
                interface->assert(NBusSignal::WriteEnable, writeMode);
                phase = BusPhase::Write;
            }
            else {
                addressInFlight = highPriorityAddress;
                interface->assert(NBusSignal::ReadEnable, 1);
                phase = BusPhase::HighRead;
            }
        }
        else if (hasLowPriority) {
            interface->assert(NBusSignal::Address, lowPriorityAddress);
            interface->assert(NBusSignal::ReadEnable, 1);
            addressInFlight = lowPriorityAddress;
            phase = BusPhase::LowRead;
        }
        readReady = false;
    }
}
void BusUnit::clockDown() {
    uint8_t interrupts = 0;
    if (interface->sense(NBusSignal::Interrupt0)) {
        interrupts |= 1 << 2;
    }
    if (interface->sense(NBusSignal::Interrupt1)) {
        interrupts |= 1 << 3;
    }
    if (interface->sense(NBusSignal::Interrupt2)) {
        interrupts |= 1 << 4;
    }
    if (interface->sense(NBusSignal::Interrupt3)) {
        interrupts |= 1 << 5;
    }
    interruptState = interrupts;

    switch (phase) {
        case BusPhase::LowRead:
        case BusPhase::HighRead:
            if (phase == BusPhase::LowRead) {
                phase = BusPhase::LowReadWait;
                hasLowPriority = false;
            }
            else {
                phase = BusPhase::HighReadWait;
                hasHighPriority = false;
            }
            break;
        case BusPhase::LowReadWait:
        case BusPhase::HighReadWait:
            if (interface->sense(NBusSignal::NotReady)) {
                break;
            }

            readAddress = addressInFlight;
            readData = (uint16_t) (interface->sense(NBusSignal::Data) & 0xffff);
            readReady = true;
            readPriority = phase == BusPhase::HighReadWait;
            phase = BusPhase::Idle;
            break;
        case BusPhase::Write:
            phase = BusPhase::WriteWait;
            hasHighPriority = false;
            hasHighPriorityWrite = false;
            break;
        case BusPhase::WriteWait:
            if (interface->sense(NBusSignal::NotReady)) {
                break;
            }

            phase = BusPhase::Idle;
            break;
        case BusPhase::Idle:
        default:
            break;
    }
}

void BusUnit::addLowPriorityRead(uint32_t address) {
    lowPriorityAddress = address;
    hasLowPriority = true;
}
void BusUnit::addHighPriorityRead(uint32_t address) {
    highPriorityAddress = address;
    hasHighPriority = true;
    hasHighPriorityWrite = false;
}
void BusUnit::addHighPriorityWrite(uint32_t address, uint16_t data, uint8_t activeBytes) {
    highPriorityAddress = address;
    highPriorityData = data;
    hasHighPriority = true;
    hasHighPriorityWrite = true;
    writeMode = activeBytes;
}

bool BusUnit::isReadReady() {
    return readReady;
}
bool BusUnit::isHighReadPriority() {
    return readPriority;
}
uint16_t BusUnit::getReadData() {
    readReady = false;
    return readData;
}
uint32_t BusUnit::getReadAddress() {
    return readAddress;
}

uint8_t BusUnit::hasInterrupt() {
    return interruptState;
}

}; // namespace n16r

}; // namespace nbus

}; // namespace sysnp

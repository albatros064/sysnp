#include "busunit.h"
#include <iostream>

namespace sysnp {

namespace nbus {

namespace n16r {

void BusUnit::setBusInterface(std::shared_ptr<NBusInterface> interface) {
    this->interface = interface;
}
void BusUnit::reset() {
    phase = BusPhase::BusIdle;
    BusOperation op;
    op.isValid = false;
    currentOperation = op;
    notReady = false;
    interruptState = 0;
    addressCounter = 0;
    dataCounter = 0;
}

void BusUnit::clockUp() {
    interface->deassertSignal(NBusSignal::Address);
    interface->deassertSignal(NBusSignal::Data);
    interface->deassertSignal(NBusSignal::WriteEnable);
    interface->deassertSignal(NBusSignal::ReadEnable);

    switch (phase) {
        case BusPhase::BusBegin:
        case BusPhase::BusWait:
            interface->assertSignal(NBusSignal::Address, (currentOperation.address + addressCounter) & 0xfffffe);
            interface->assertSignal(NBusSignal::ReadEnable, readMode);
            interface->assertSignal(NBusSignal::WriteEnable, writeMode);
            if (!currentOperation.isRead) {
                interface->assertSignal(NBusSignal::Data, currentOperation.data[dataCounter]);
            }
            break;
        case BusPhase::BusActive:
            if (!currentOperation.isRead) {
                interface->assertSignal(NBusSignal::Data, currentOperation.data[dataCounter]);
            }
            interface->assertSignal(NBusSignal::Address, (currentOperation.address + addressCounter) & 0xfffffe);
            interface->assertSignal(NBusSignal::ReadEnable, readMode);
            interface->assertSignal(NBusSignal::WriteEnable, writeMode);
            break;
        case BusPhase::BusCleanup:
            if (!currentOperation.isRead) {
                interface->assertSignal(NBusSignal::Data, currentOperation.data[dataCounter]);
            }
            break;
        case BusPhase::BusIdle:
        default:
            break;
    }
}

void BusUnit::clockDown() {
    uint8_t interrupts = 0;
    if (interface->senseSignal(NBusSignal::Interrupt0)) {
        interrupts |= 1 << 2;
    }
    if (interface->senseSignal(NBusSignal::Interrupt1)) {
        interrupts |= 1 << 3;
    }
    if (interface->senseSignal(NBusSignal::Interrupt2)) {
        interrupts |= 1 << 4;
    }
    if (interface->senseSignal(NBusSignal::Interrupt3)) {
        interrupts |= 1 << 5;
    }
    interruptState = interrupts;

    notReady = interface->senseSignal(NBusSignal::NotReady) > 0;
    uint16_t data = (uint16_t) (interface->senseSignal(NBusSignal::Data) & 0xffff);

    switch (phase) {
        case BusPhase::BusBegin:
            phase = BusPhase::BusWait;
            break;
        case BusPhase::BusWait:
            if (!notReady) {
                phase = BusPhase::BusActive;
                addressCounter += 2;
                currentOperation.bytes -= 2;
                if (!currentOperation.isRead) {
                    dataCounter += 1;
                    if (currentOperation.bytes <= 0) {
                        phase = BusPhase::BusCleanup;
                    }
                }
            }
            break;
        case BusPhase::BusActive:
            if (notReady) {
                phase = BusPhase::BusWait;
                break;
            }

            addressCounter += 2;
            dataCounter    += 1;

            if (currentOperation.isRead) {
                currentOperation.data.push_back(data);
            }

            currentOperation.bytes -= 2;
            if (currentOperation.bytes <= 0) {
                phase = BusPhase::BusCleanup;
            }
            break;
        case BusPhase::BusCleanup:
            if (currentOperation.isRead) {
                currentOperation.data.push_back(data);
            }

            phase = BusPhase::BusIdle;
            break;
        case BusPhase::BusIdle:
        default:
            if (currentOperation.isValid && currentOperation.bytes > 0) {
                phase = BusPhase::BusBegin;
                addressCounter = 0;
                dataCounter = -1;
                readMode = currentOperation.bytes > 2 ? 0b10 : 0b01;
                if (currentOperation.isRead) {
                    writeMode = 0b00;
                }
                else if (currentOperation.bytes == 1) {
                    writeMode = (currentOperation.address & 1) ? 0b10 : 0b01;
                }
                else {
                    writeMode = 0b11;
                }
            }
            break;
    }
}

bool BusUnit::isIdle() {
    return phase == BusPhase::BusIdle && !notReady;
}

bool BusUnit::hasData() {
    return currentOperation.isRead && currentOperation.isValid && currentOperation.data.size() > 0;
}

uint16_t BusUnit::getWord() {
    uint16_t word = 0;
    if (currentOperation.isValid && currentOperation.data.size() > 0) {
        word = currentOperation.data[0];
        currentOperation.data.erase(currentOperation.data.begin());

        if (currentOperation.data.size() <= 0 && currentOperation.bytes < 0) {
            currentOperation.isValid = false;
        }
    }
    return word;
}

void BusUnit::queueOperation(BusOperation operation) {
    currentOperation = operation;
}

uint8_t BusUnit::hasInterrupt() {
    return interruptState;
}

}; // namespace n16r

}; // namespace nbus

}; // namespace sysnp

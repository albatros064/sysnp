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
    interruptState = 0;
}

void BusUnit::clockUp() {
    interface->deassertSignal(NBusSignal::Address);
    interface->deassertSignal(NBusSignal::Data);
    interface->deassertSignal(NBusSignal::WriteEnable);
    interface->deassertSignal(NBusSignal::ReadEnable);

    if (phase == BusPhase::BusIdle) {
        if (currentOperation.isValid) {
            bool upperByte = currentOperation.address & 1;
            interface->assertSignal(NBusSignal::Address, currentOperation.address & 0xfffffe);

            if (currentOperation.isRead) {
                uint8_t readSignal = 0b01;
                if (currentOperation.bytes > 2) {
                    readSignal = 0b11;
                }
                interface->assertSignal(NBusSignal::ReadEnable, readSignal);

                phase = BusPhase::BusRead;
            }
            else {
                uint8_t writeMode = 0b11;
                if (currentOperation.bytes == 1) {
                    writeMode = upperByte ? 0b10 : 0b01;
                }
                interface->assertSignal(NBusSignal::WriteEnable, writeMode);
                interface->assertSignal(NBusSignal::Data, currentOperation.data[0]);

                phase = BusPhase::BusWrite;
            }
        }
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

    bool notReady =             interface->senseSignal(NBusSignal::NotReady) > 0;
    uint16_t data = (uint16_t) (interface->senseSignal(NBusSignal::Data    ) & 0xffff);

    switch (phase) {
        case BusPhase::BusRead:
            phase = BusPhase::BusReadWait;
            break;
        case BusPhase::BusReadWait:
            if (notReady) {
                break;
            }

            currentOperation.data.push_back(data);
            currentOperation.bytes -= 2;
            if (currentOperation.bytes <= 0) {
                phase = BusPhase::BusIdle;
            }

            break;
        case BusPhase::BusWrite:
            phase = BusPhase::BusWriteWait;

            break;
        case BusPhase::BusWriteWait:
            phase = BusPhase::BusIdle;

            break;
        case BusPhase::BusIdle:
        default:
            break;
    }
}

bool BusUnit::isIdle() {
    return phase == BusPhase::BusIdle;
}

bool BusUnit::hasData() {
    return currentOperation.isRead && currentOperation.isValid && currentOperation.data.size() > 0;
}

uint16_t BusUnit::getWord() {
    uint16_t word = 0;
    if (currentOperation.isValid && currentOperation.data.size() > 0) {
        word = currentOperation.data[0];
        currentOperation.data.erase(currentOperation.data.begin());

        if (currentOperation.data.size() == 0 && currentOperation.bytes == 0) {
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

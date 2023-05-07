#include "nbus.h"

#include <iostream>
#include <sstream>
#include <iomanip>

namespace sysnp {

NBusInterface::NBusInterface(std::shared_ptr<NBus> inBus, std::shared_ptr<Device> inDevice):
        bus(inBus),
        device(inDevice) {
    for (unsigned i = 0; i <= NBusSignal::NotReady; i++) {
        signals[i] = 0;
    }
}

void NBusInterface::assert(NBusSignal signal, uint32_t value) {
    signals[signal] = value;
}
void NBusInterface::deassert(NBusSignal signal) {
    assert(signal, 0);
}
uint32_t NBusInterface::sense(NBusSignal signal) {
    return bus->sense(signal);
}

void NBusInterface::clockUp() {
    device->clockUp();
}
void NBusInterface::clockDown() {
    device->clockDown();
}

NBus::NBus() {
    signalMasks[NBusSignal::Address] = 0xffffff;
    signalMasks[NBusSignal::Data] = 0xffff;
    signalMasks[NBusSignal::WriteEnable] = 3;
    signalMasks[NBusSignal::ReadEnable ] = 3;
    signalMasks[NBusSignal::Interrupt0] = 1;
    signalMasks[NBusSignal::Interrupt1] = 1;
    signalMasks[NBusSignal::Interrupt2] = 1;
    signalMasks[NBusSignal::Interrupt3] = 1;
    signalMasks[NBusSignal::NotReady] = 1;
}

void NBus::init(std::shared_ptr<Machine> machine, const libconfig::Setting &setting) {
    this->machine = machine;
    const libconfig::Setting &devices = setting["devices"];
    int deviceCount = devices.getLength();
    for (int i = 0; i < deviceCount; i++) {
        deviceNames.push_back(devices[i]);
    }
}
void NBus::postInit() {
    for (auto deviceName: deviceNames) {
        machine->debug("NBus: Finding " + deviceName);
        std::shared_ptr<NBusDevice> device = std::static_pointer_cast<NBusDevice>(machine->getDevice(deviceName));
        if (device) {
            std::shared_ptr<NBusInterface> interface = std::make_shared<NBusInterface>(shared_from_this(), device);
            device->setInterface(interface);
            addInterface(interface);
        }
    }
    selfInterface = std::make_shared<NBusInterface>(shared_from_this(), shared_from_this());
}

std::shared_ptr<NBusInterface> NBus::getIndependentInterface() {
    return selfInterface;
}

uint32_t NBus::sense(NBusSignal signal) {
    uint32_t signalValue = 0;

    for (auto interface: interfaces) {
        signalValue |= interface->signals[signal];
    }

    signalValue |= selfInterface->signals[signal];

    signalValue &= signalMasks[signal];

    return signalValue;
}

void NBus::clockUp() {
    for (auto interface: interfaces) {
        interface->clockUp();
    }
}
void NBus::clockDown() {
    for (auto interface: interfaces) {
        interface->clockDown();
    }
}

std::string NBus::output(uint8_t mode) {
    return "";
}

void NBus::addInterface(std::shared_ptr<NBusInterface> interface) {
    interfaces.push_back(interface);
}

};


#include "nbus.h"

#include <iostream>
#include <bitset>
#include <sstream>
#include <iomanip>

namespace sysnp {

namespace nbus {

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

void NBus::init(const libconfig::Setting &setting) {
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

std::string NBus::command(std::stringstream &input) {
    std::string commandWord = "all";
    input >> commandWord;

    uint32_t setValue = 0xffffffff;
    std::stringstream stream;

    if (commandWord == "r" || commandWord == "release") {
        selfInterface->deassert(NBusSignal::Address);
        selfInterface->deassert(NBusSignal::Data);
        selfInterface->deassert(NBusSignal::ReadEnable);
        selfInterface->deassert(NBusSignal::WriteEnable);
        selfInterface->deassert(NBusSignal::Interrupt0);
        selfInterface->deassert(NBusSignal::Interrupt1);
        selfInterface->deassert(NBusSignal::Interrupt2);
        selfInterface->deassert(NBusSignal::Interrupt3);
        selfInterface->deassert(NBusSignal::NotReady);
        stream << "Ok.";
    }
    else if (commandWord == "address") {
        input >> setValue;
        if (setValue != 0xffffffff) {
            selfInterface->assert(NBusSignal::Address, setValue);
            stream << "Ok.";
        }
        else {
            stream << "0" << std::setw(8) << std::setfill('0') << std::oct;
            stream << selfInterface->sense(NBusSignal::Address);
        }
    }
    else if (commandWord == "data") {
        input >> setValue;
        if (setValue != 0xffffffff) {
            selfInterface->assert(NBusSignal::Data, setValue);
            stream << "Ok.";
        }
        else {
            stream << "0" << std::setw(6) << std::setfill('0') << std::oct;
            stream << selfInterface->sense(NBusSignal::Data);
        }
    }
    else if (commandWord == "read") {
        input >> setValue;
        if (setValue != 0xffffffff) {
            selfInterface->assert(NBusSignal::ReadEnable, setValue);
            stream << "Ok.";
        }
        else {
            stream << "0b" << std::bitset<2>(selfInterface->sense(NBusSignal::ReadEnable));
        }
    }
    else if (commandWord == "write") {
        input >> setValue;
        if (setValue != 0xffffffff) {
            selfInterface->assert(NBusSignal::WriteEnable, setValue);
            stream << "Ok.";
        }
        else {
            stream << "0b" << std::bitset<2>(selfInterface->sense(NBusSignal::WriteEnable));
        }
    }
    else {
        stream << "ADDR----- DATA--- WR RD INT- R" << std::endl;
        stream << "0" << std::setfill('0') << std::setw(8) << std::oct << selfInterface->sense(NBusSignal::Address    ) << " ";
        stream << "0" << std::setw(6) << selfInterface->sense(NBusSignal::Data       ) << " " << std::setbase(10);
        stream << std::bitset<2>(selfInterface->sense(NBusSignal::WriteEnable)) << " ";
        stream << std::bitset<2>(selfInterface->sense(NBusSignal::ReadEnable )) << " ";
        stream << selfInterface->sense(NBusSignal::Interrupt0) << selfInterface->sense(NBusSignal::Interrupt1);
        stream << selfInterface->sense(NBusSignal::Interrupt2) << selfInterface->sense(NBusSignal::Interrupt3) << " ";
        stream << selfInterface->sense(NBusSignal::NotReady);
    }
    return stream.str();
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

void NBus::addInterface(std::shared_ptr<NBusInterface> interface) {
    interfaces.push_back(interface);
}

}; // namespace nbus

}; // namespace sysnp


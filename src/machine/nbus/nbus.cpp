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

void NBusInterface::assertSignal(NBusSignal signal, uint32_t value) {
    signals[signal] = value;
}
void NBusInterface::deassertSignal(NBusSignal signal) {
    assertSignal(signal, 0);
}
uint32_t NBusInterface::senseSignal(NBusSignal signal) {
    return bus->senseSignal(signal);
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
    signalMasks[NBusSignal::WriteEnable] = 0b11;
    signalMasks[NBusSignal::ReadEnable ] = 0b11;
    signalMasks[NBusSignal::Interrupt0] = 1;
    signalMasks[NBusSignal::Interrupt1] = 1;
    signalMasks[NBusSignal::Interrupt2] = 1;
    signalMasks[NBusSignal::Interrupt3] = 1;
    signalMasks[NBusSignal::NotReady] = 1;
}

void NBus::init(ryml::NodeRef &setting) {
    this->machine = machine;
    auto devicesConfig = setting["devices"];
    int deviceCount = devicesConfig.num_children();
    for (int i = 0; i < deviceCount; i++) {
        std::string deviceName;
        devicesConfig[i] >> deviceName;
        deviceNames.push_back(deviceName);
    }
}
void NBus::postInit() {
    machine->debug("NBus::postInit()");
    for (auto deviceName: deviceNames) {
        machine->debug(" -Finding " + deviceName);
        std::shared_ptr<NBusDevice> device = std::static_pointer_cast<NBusDevice>(machine->getDevice(deviceName));
        if (device) {
            machine->debug(" --Initializing");
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
        selfInterface->deassertSignal(NBusSignal::Address);
        selfInterface->deassertSignal(NBusSignal::Data);
        selfInterface->deassertSignal(NBusSignal::ReadEnable);
        selfInterface->deassertSignal(NBusSignal::WriteEnable);
        selfInterface->deassertSignal(NBusSignal::Interrupt0);
        selfInterface->deassertSignal(NBusSignal::Interrupt1);
        selfInterface->deassertSignal(NBusSignal::Interrupt2);
        selfInterface->deassertSignal(NBusSignal::Interrupt3);
        selfInterface->deassertSignal(NBusSignal::NotReady);
        stream << "Ok.";
    }
    else if (commandWord == "address") {
        input >> setValue;
        if (setValue != 0xffffffff) {
            selfInterface->assertSignal(NBusSignal::Address, setValue);
            stream << "Ok.";
        }
        else {
            stream << "0" << std::setw(8) << std::setfill('0') << std::oct;
            stream << selfInterface->senseSignal(NBusSignal::Address);
        }
    }
    else if (commandWord == "data") {
        input >> setValue;
        if (setValue != 0xffffffff) {
            selfInterface->assertSignal(NBusSignal::Data, setValue);
            stream << "Ok.";
        }
        else {
            stream << "0" << std::setw(6) << std::setfill('0') << std::oct;
            stream << selfInterface->senseSignal(NBusSignal::Data);
        }
    }
    else if (commandWord == "read") {
        input >> setValue;
        if (setValue != 0xffffffff) {
            selfInterface->assertSignal(NBusSignal::ReadEnable, setValue);
            stream << "Ok.";
        }
        else {
            stream << "0b" << std::bitset<2>(selfInterface->senseSignal(NBusSignal::ReadEnable));
        }
    }
    else if (commandWord == "write") {
        input >> setValue;
        if (setValue != 0xffffffff) {
            selfInterface->assertSignal(NBusSignal::WriteEnable, setValue);
            stream << "Ok.";
        }
        else {
            stream << "0b" << std::bitset<2>(selfInterface->senseSignal(NBusSignal::WriteEnable));
        }
    }
    else {
        stream << "ADDR---- DATA-- WR RD INT- H" << std::endl;
        stream << "0x" << std::setfill('0') << std::setw(6) << std::hex << selfInterface->senseSignal(NBusSignal::Address) << " ";
        stream << "0x" << std::setfill('0') << std::setw(4) << std::hex << selfInterface->senseSignal(NBusSignal::Data) << " " << std::setbase(10);
        stream << std::bitset<2>(selfInterface->senseSignal(NBusSignal::WriteEnable)) << " ";
        stream << std::bitset<2>(selfInterface->senseSignal(NBusSignal::ReadEnable )) << " ";
        stream << selfInterface->senseSignal(NBusSignal::Interrupt0) << selfInterface->senseSignal(NBusSignal::Interrupt1);
        stream << selfInterface->senseSignal(NBusSignal::Interrupt2) << selfInterface->senseSignal(NBusSignal::Interrupt3) << " ";
        stream << selfInterface->senseSignal(NBusSignal::NotReady);
    }
    return stream.str();
}

std::shared_ptr<NBusInterface> NBus::getIndependentInterface() {
    return selfInterface;
}

uint32_t NBus::senseSignal(NBusSignal signal) {
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


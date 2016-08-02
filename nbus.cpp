#include "nbus.h"

namespace sysnp {

NBus::NBus() {}

void NBus::init(Machine *machine, const libconfig::Setting &config) {
    this->machine = machine;
    machine->debug("NBus::init()");
    const libconfig::Setting &devices = config["devices"];
    int connectedDevices = devices.getLength();
    for (int i = 0; i < connectedDevices; i++) {
        string deviceName = devices[i];
        deviceNames.push_back(deviceName);
    }

    deviceAddress = config["device"];
    clockFrequency = config["clock"];
}

void NBus::postInit() {
    machine->debug("NBus::postInit()");
    for (string deviceName: deviceNames) {
        Device *device = machine->getDevice(deviceName);
        NBusDevice *busDevice = dynamic_cast<NBusDevice *>(device);
        machine->debug(" Device \"" + deviceName + "\" ");
        if (busDevice) {
            machine->debug("  Found");
            devices.push_back(busDevice);
        }
        else if (device) {
            machine->debug("  Found, but not an NBusDevice. Ignoring.");
        }
        else {
            machine->debug("  Not found. Ignoring.");
        }
    }

    for (int s = Signal::Address; s != Signal::InterruptD; s++) {
        drive(static_cast<Signal>(s), 0);
    }
}

uint32_t NBus::busAddress(bool set, uint32_t value) {
    if (set) {
        drive(Address, value & 0x00ffffff);
    }
    return sense(Address);
}
uint16_t NBus::busData(bool set, uint16_t value) {
    if (set) {
        drive(Data, value);
    }
    return (uint16_t) sense(Data);
}

bool NBus::busRead(bool set, bool value) {
    if (set) {
        drive(ReadEnable, value ? 0 : 1);
    }
    return sense(ReadEnable) > 0 ? true : false;
}
bool NBus::busWrite(bool set, bool value) {
    if (set) {
        drive(WriteEnable, value ? 0 : 1);
    }
    return sense(WriteEnable) > 0 ? true : false;
}

void NBus::drive(Signal signal, uint32_t value) {
    nextSignals[signal] = value;
}
uint32_t NBus::sense(Signal signal) {
    return signals[signal];
}

void NBus::clock() {
    signals = nextSignals;
    for (NBusDevice *device: devices) {
        device->clock(*this);
    }
}

};


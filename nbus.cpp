#include <iostream>
#include "nbus.h"

namespace sysnp {

void NBus::init(Machine &machine, const libconfig::Setting &config) {
    std::cout << "NBus::init()" << std::endl;
    const libconfig::Setting &devices = config["devices"];
    int connectedDevices = devices.getLength();
    for (int i = 0; i < connectedDevices; i++) {
        string deviceName = devices[i];
        std::cout << "-" << deviceName << std::endl;
        deviceNames.push_back(deviceName);
    }

    deviceAddress = config["device"];
    clockFrequency = config["clock"];
}

void NBus::postInit(Machine &machine) {
    for (string deviceName: deviceNames) {
        Device *device = machine.getDevice(deviceName);
        NBusDevice *busDevice = dynamic_cast<NBusDevice *>(device);
        if (busDevice) {
            devices.push_back(busDevice);
        }
        else if (device) {
            std::cout << "NBus: device \"" << deviceName << "\" found, but is not an NBusDevice. Ignoring." << std::endl;
        }
        else {
            std::cout << "NBus: device \"" << deviceName << "\" not found. Ignoring." << std::endl;
        }
    }
}

void NBus::drive(uint8_t signal, uint32_t value) {
    signals[signal] = value;
}
uint32_t NBus::sense(uint8_t signal) {
    return signals[signal];
}

void NBus::clock() {
    for (NBusDevice *device: devices) {
        device->clock(*this);
    }
}

};


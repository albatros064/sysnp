#include <libconfig.h++>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <fstream>

#include "machine.h"
#include "device.h"
#include "nbus/nbus.h"
#include "nbus/memory.h"
#include "nbus/serial.h"
#include "nbus/cpu/n16r.h"

namespace sysnp {

bool Machine::load(std::string configFile) {
    debugLevel = -1;
    std::string rootDeviceName = "none";
    try {
        libconfig::Config config;
        config.readFile(configFile.c_str());

        const libconfig::Setting& configRoot = config.getRoot();

        configRoot.lookupValue("root", rootDeviceName);

        configRoot.lookupValue("debugLevel", debugLevel);

        const libconfig::Setting& cDevices = configRoot["devices"];
        int deviceCount = cDevices.getLength();
        for (int i = 0; i < deviceCount; i++) {
            std::string moduleName;
            const libconfig::Setting& cDevice = cDevices[i];
            cDevice.lookupValue("module", moduleName);

            debug("Creating device type \"" + moduleName + "\"");

            std::shared_ptr<Device> newDevice = createDevice(moduleName);
            if (newDevice) {
                debug("Initializing device type \"" + moduleName + "\"");
                newDevice->setMachine(shared_from_this());
                newDevice->init(cDevice);
                devices[moduleName] = newDevice;
            }
        }
    }
    catch (libconfig::ParseException e) {
        debugLevel = 1;
        debug(1, "Error in config file \"" + configFile + "\" on line " + std::to_string(e.getLine()) +  ": " + e.getError());
        return false;
    }

    debug("PostInit phase");

    auto rootDevice = devices.find(rootDeviceName);
    if (rootDevice != devices.end()) {
        debug(": " + rootDeviceName);
        rootDevice->second->postInit();
    }

    for (auto iter: devices) {
        if (iter.first == rootDeviceName) {
            continue;
        }

        debug(": " + iter.first);
        if (iter.second) {
            iter.second->postInit();
        }
    }

    
    debug("Done loading");

    return true;
}

std::shared_ptr<Device> Machine::createDevice(std::string deviceName) {
    std::shared_ptr<Device> newDevice;
    if (deviceName == "n16r") {
        newDevice = std::make_shared<nbus::n16r::N16R>();
    }
    else if (deviceName == "memory") {
        newDevice = std::make_shared<nbus::Memory>();
    }
    else if (deviceName == "nbus") {
        newDevice = std::make_shared<nbus::NBus>();
    }
    else if (deviceName == "serial") {
        newDevice = std::make_shared<nbus::Serial>();
    }
    return newDevice;
}

std::shared_ptr<Device> Machine::getDevice(std::string deviceName) {
    auto iter = devices.find(deviceName);
    if (iter != devices.end()) {
        return iter->second;
    }

    return 0;
}

bool Machine::readFile(std::string fileName, uint8_t *dest, uint32_t limit) {
    std::ifstream fs;
    fs.open(fileName);
    std::filebuf *buffer = fs.rdbuf();

    buffer->sgetn((char*)dest, limit);

    fs.close();

    return true;
}

void Machine::debug(std::string message) {
    debug(3, message);
}
void Machine::debug(int level, std::string message) {
    if (debugLevel >= level) {
        std::cout << "DEBUG[" << level << "]: " << message << std::endl;
    }
}

void Machine::run() {
    char commandBuffer[255];
    debug("");
    debug("Fetching bus device");
    std::shared_ptr<nbus::NBus> bus = std::static_pointer_cast<nbus::NBus>(getDevice("nbus"));

    bool verbose = false;
    std::string command;
    std::string commandWord;
    bool running = true;

    debug("Entering loop");
    debug("");

    RunMode runMode = RunMode::SteppingMode;
    while (running) {
        std::cout << "> ";
        std::cin.getline(commandBuffer, 255);
        std::stringstream cs(commandBuffer);

        cs >> command;

        if (command == "q" || command == "quit") {
            running = false;
            stopRunning();
        }
        else if (command == "b" || command == "bp") {
        }
        else if (runMode == RunMode::SteppingMode) {
            if (command == "pulse") {
                commandWord = "1";
                cs >> commandWord;

                int repeatCount = 1;
                try {
                    repeatCount = std::stoi(commandWord);
                }
                catch (std::invalid_argument e) {}

                for (; repeatCount > 0; repeatCount--) {
                    bus->clockUp();
                    bus->clockDown();
                }
            }
            else if (command == "run") {
                commandWord = "0";
                cs >> commandWord;

                runMode = RunMode::FreeRunMode;

                int clockRate = 0;

                try {
                    clockRate = std::stoi(commandWord);
                }
                catch (std::invalid_argument e) {}

                clockRunning = true;
                runThread = std::thread(machineRun, std::ref(*this), clockRate);
            }
            else if (command == "bus") {
                std::cout << bus->command(cs) << std::endl;
            }
            else if (command == "dev") {
                commandWord = "";
                cs >> commandWord;
                if (commandWord.empty()) {
                    std::cout << "Invalid device." << std::endl;
                }
                else {
                    auto device = getDevice(commandWord);
                    if (device) {
                        std::cout << device->command(cs) << std::endl;
                    }
                    else {
                        std::cout << "Invalid device." << std::endl;
                    }
                }
            }
            else if (command == "config") {
                cs >> commandWord;
                if (commandWord == "debug") {
                    int newDebug = -1;
                    cs >> newDebug;
                    if (newDebug >= 0) {
                        debugLevel = newDebug;
                    }
                }
            }
        }
        else if (runMode == RunMode::FreeRunMode) {
            if (command == "p" || command == "pause") {
                stopRunning();
                runMode = RunMode::SteppingMode;
            }
        }
        else if (command != "") {
            std::cout << "Unrecognized command: '" << cs.str() << "'" << std::endl;
        }
    }

    debug("Exiting");
}

void Machine::startRunning(int clockRate) {
    std::shared_ptr<nbus::NBus> bus = std::static_pointer_cast<nbus::NBus>(getDevice("nbus"));

    runCycles = 0;
    runStart = std::chrono::steady_clock::now();
    while (clockRunning) {
        bus->clockUp();
        bus->clockDown();

        runCycles++;
    }
    runEnd = std::chrono::steady_clock::now();
}
void Machine::stopRunning() {
    clockRunning = false;
    if (runThread.joinable()) {
        runThread.join();
    }

    auto diff = std::chrono::nanoseconds(runEnd - runStart).count();
    std::cout << "ticks: " << runCycles << std::endl;
    std::cout << "ns:    " << diff << std::endl;
    std::cout << "       " << (runCycles / ((double) diff / 1000000000)) << "Hz" << std::endl;
}

void machineRun(Machine& machine, int clockRate) {
    machine.startRunning(clockRate);
}

}; // namespace

#include <libconfig.h++>
#include <iostream>
#include <iomanip>
#include <bitset>
#include <fstream>

#include "machine.h"
#include "device.h"
#include "nbus.h"
#include "memory.h"
#include "util.h"

namespace sysnp {

bool Machine::load(std::string configFile) {
    debugLevel = -1;
    try {
        libconfig::Config config;
        config.readFile(configFile.c_str());

        const libconfig::Setting& configRoot = config.getRoot();

        configRoot.lookupValue("debugLevel", debugLevel);

        const libconfig::Setting& cDevices = configRoot["devices"];
        int deviceCount = cDevices.getLength();
        for (int i = 0; i < deviceCount; i++) {
            std::string moduleName;
            const libconfig::Setting& cDevice = cDevices[i];
            cDevice.lookupValue("module", moduleName);

            debug("Creating device type \"" + moduleName + "\"");

            std::shared_ptr<Device> newDevice = this->createDevice(moduleName);
            if (newDevice) {
                debug("Initializing device type \"" + moduleName + "\"");
                newDevice->init(shared_from_this(), cDevice);
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

    for (auto iter: devices) {
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
    if (deviceName.compare("np16") == 0) {
        //newDevice = std::make_shared<Np16>();
    }
    else if (deviceName.compare("memory") == 0) {
        newDevice = std::make_shared<Memory>();
    }
    else if (deviceName.compare("nbus") == 0) {
        newDevice = std::make_shared<NBus>();
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
    std::shared_ptr<NBus> bus = std::static_pointer_cast<NBus>(getDevice("nbus"));
    std::shared_ptr<NBusInterface> busInterface = bus->getIndependentInterface();

    std::string command;
    std::string commandWord;
    bool running = true;

    debug("Entering loop");
    debug("");

    bool clockState = false;
    while (running) {
        std::cout << "> ";
        std::cin.getline(commandBuffer, 255);
        std::stringstream cs(commandBuffer);

        cs >> command;

        if (command == "q" || command == "quit") {
            running = false;
        }
        else if (command == "clock") {
            if (!std::getline(cs, commandWord, ' ')) {
                commandWord = "pulse";
            }

            if (commandWord == "pulse") {
                bus->clockUp();
                bus->clockDown();
            }
            else if (commandWord == "rise") {
                bus->clockUp();
            }
            else if (commandWord == "fall") {
                bus->clockDown();
            }
            else if (commandWord == "set") {
                if (!clockState) {
                    bus->clockUp();
                    clockState = true;
                }
            }
            else if (commandWord == "clear") {
                if (clockState) {
                    bus->clockDown();
                    clockState = false;
                }
            }
        }
        else if (command == "bus") {
            commandWord = "all";
            cs >> commandWord;

            uint32_t setValue = 0xffffffff;
            std::stringstream stream;

            if (commandWord == "r" || commandWord == "release") {
                busInterface->deassert(NBusSignal::Address);
                busInterface->deassert(NBusSignal::Data);
                busInterface->deassert(NBusSignal::ReadEnable);
                busInterface->deassert(NBusSignal::WriteEnable);
                busInterface->deassert(NBusSignal::Interrupt0);
                busInterface->deassert(NBusSignal::Interrupt1);
                busInterface->deassert(NBusSignal::Interrupt2);
                busInterface->deassert(NBusSignal::Interrupt3);
                busInterface->deassert(NBusSignal::NotReady);
                stream << "Ok.";
            }
            else if (commandWord == "address") {
                cs >> setValue;
                if (setValue != 0xffffffff) {
                    busInterface->assert(NBusSignal::Address, setValue);
                    stream << "Ok.";
                }
                else {
                    stream << "0" << std::setw(8) << std::setfill('0') << std::oct;
                    stream << busInterface->sense(NBusSignal::Address);
                }
            }
            else if (commandWord == "data") {
                cs >> setValue;
                if (setValue != 0xffffffff) {
                    busInterface->assert(NBusSignal::Data, setValue);
                    stream << "Ok.";
                }
                else {
                    stream << "0" << std::setw(6) << std::setfill('0') << std::oct;
                    stream << busInterface->sense(NBusSignal::Data);
                }
            }
            else if (commandWord == "read") {
                cs >> setValue;
                if (setValue != 0xffffffff) {
                    busInterface->assert(NBusSignal::ReadEnable, setValue);
                    stream << "Ok.";
                }
                else {
                    stream << "0b" << std::bitset<2>(busInterface->sense(NBusSignal::ReadEnable));
                }
            }
            else if (commandWord == "write") {
                cs >> setValue;
                if (setValue != 0xffffffff) {
                    busInterface->assert(NBusSignal::WriteEnable, setValue);
                    stream << "Ok.";
                }
                else {
                    stream << "0b" << std::bitset<2>(busInterface->sense(NBusSignal::WriteEnable));
                }
            }
            else {
                stream << "ADDR----- DATA--- WR RD INT- R" << std::endl;
                stream << "0" << std::setfill('0') << std::setw(8) << std::oct << busInterface->sense(NBusSignal::Address    ) << " ";
                stream << "0" << std::setw(6) << busInterface->sense(NBusSignal::Data       ) << " " << std::setbase(10);
                stream << std::bitset<2>(busInterface->sense(NBusSignal::WriteEnable)) << " ";
                stream << std::bitset<2>(busInterface->sense(NBusSignal::ReadEnable )) << " ";
                stream << busInterface->sense(NBusSignal::Interrupt0) << busInterface->sense(NBusSignal::Interrupt1);
                stream << busInterface->sense(NBusSignal::Interrupt2) << busInterface->sense(NBusSignal::Interrupt3) << " ";
                stream << busInterface->sense(NBusSignal::NotReady);
            }
            std::cout << stream.str() << std::endl;
        }
        else if (command == "dev") {
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
        else if (command != "") {
            std::cout << "Unrecognized command: '" << cs.str() << "'" << std::endl;
        }
    }

    debug("Exiting");
}

/*

void Machine::run() {
	uint8_t instr_length;
	uint8_t bytes[3] = { _cpu->data_bits() / 8, _cpu->logical_bits() / 8, _cpu->physical_bits() / 8 };

	char buffer[16];

	uint16_t  register_count[4] = { 0, 0, 0, 0 };
	uint64_t *registers[2];

	registers[0] = _cpu->register_file(register_count[0]);
	registers[1] = _cpu->register_aux (register_count[1]);

	register_count[2] = register_count[0] / 16; register_count[0] % 16 ? register_count[2]++ : register_count[2];
	register_count[3] = register_count[1] / 16; register_count[1] % 16 ? register_count[3]++ : register_count[3];

	char *register_filler = (char *) malloc(bytes[0] * 2 + 8);
	int i;
	for (i = 0; i < bytes[0] * 2 + 7; i++) {
		register_filler[i] = ' ';
	}
	register_filler[i] = 0;

	_cpu->reset();
	do {
		uint64_t instr_addr = _cpu->step(instr_length);

		printf("\n----\n PC:    0x%0*lx\n Instr: 0x", bytes[1] * 2, instr_addr);
		for(int i = 0; i < instr_length; i++) {
			printf("%02x", _memory[instr_addr + i]);
		}
		printf("\n        %s\n----\n Registers:\n", _cpu->decode(instr_addr) );
		for (int i = 0; i < 16; i++) {
			for (int j = 0; j < register_count[2]; j++) {
				if (j * 16 + i < register_count[0]) {
					printf(" %03d: 0x%0*lx", j * 16 + i, bytes[0] * 2, registers[0][j * 16 + i]);
				}
				else {
					printf(" %s", register_filler);
				}
			}

			printf(" |");
			for (int j = 0; j < register_count[3]; j++) {
				if (j * 16 + i < register_count[1]) {
					printf(" %03d: 0x%0*lx", j * 16 + i, bytes[0] * 2, registers[1][j * 16 + i]);
				}
				else {
					printf(" %s", register_filler);
				}
			}
			printf("\n");
		}

		printf("----\n Memory:\n");
		uint64_t stack_pointer = _cpu->stack_pointer() % _memory_size;
		uint64_t stack_pointer_min = stack_pointer - stack_pointer % 16;
		uint64_t stack_pointer_max = stack_pointer_min + 256;
		if (stack_pointer_max >= _memory_size) {
			stack_pointer_max = _memory_size - 16;
		}

		for (int i = 0; i < 16; i++) {
			uint64_t _mem = _cpu->instr_pointer() + i * 16;
			uint64_t _stk = (stack_pointer_max - i * 16);
			_mem -= _mem % 16;
			printf(" 0x%0*lx:", bytes[0] * 2, _mem);
			for (int j = 0; j < 16; j++) {
				printf(" %02x", _memory[_mem + j]);
			}

			printf("  | ");

			if (_stk >= stack_pointer_min) {
				printf(" 0x%0*lx:", bytes[0] * 2, _stk);

				for (int j = 15; j >= 0; j--) {
					if (_stk + j < stack_pointer) {
						printf("   ");
					}
					else {
						printf(" %02x", _memory[_stk + j]);
					}
				}
			}

			printf("\n");
		}//
		printf("----\n");

		gets(buffer);
		if (buffer[0] == 'r') {
			_cpu->reset();
		}
	}
	while (buffer[0] != 'q');
}
*/

}; // namespace

#ifndef SYSNP_MACHINE_H
#define SYSNP_MACHINE_H

#include <string>
#include <map>

#include "device.h"

namespace sysnp {

class Device;

class Machine {
  public:
	Machine() {}
	virtual ~Machine() {}

    bool load(std::string);

    Device *getDevice(std::string);

	void run();
  private:
    std::map<std::string,Device *> devices;

    Device *instanciateDevice(std::string);
};

}; // namepsace

#endif


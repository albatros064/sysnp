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

    bool readFile(std::string,uint8_t*,uint32_t);

	void run();

    void debug(int,std::string);
    void debug(std::string);
  private:
    std::map<std::string,Device *> devices;
    int debugLevel;

    Device *instanciateDevice(std::string);
};

}; // namepsace

#endif


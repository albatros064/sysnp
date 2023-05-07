#ifndef SYSNP_MACHINE_H
#define SYSNP_MACHINE_H

#include <string>
#include <memory>
#include <map>

#include "device.h"

namespace sysnp {

class Device;

class Machine : public std::enable_shared_from_this<Machine> {
  public:
	Machine() {}
	virtual ~Machine() {}

    bool load(std::string);

    std::shared_ptr<Device> getDevice(std::string);

    bool readFile(std::string,uint8_t*,uint32_t);

	void run();

    void debug(int,std::string);
    void debug(std::string);
  private:
    std::map<std::string,std::shared_ptr<Device>> devices;
    int debugLevel;

    std::shared_ptr<Device> createDevice(std::string);
};

}; // namepsace

#endif


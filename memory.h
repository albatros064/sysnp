#ifndef SYSNP_MEMORY_H
#define SYSNP_MEMORY_H

#include <string>

#include "nbus.h"

namespace sysnp {

class Memory : public NBusDevice {
  public:
    Memory();
    virtual ~Memory();

    // Device::init()
    virtual void init(Machine *, const libconfig::Setting &);
    // Device::postInit()
    virtual void postInit();

    // NBusDevice::clock()
    virtual void clock(NBus &);

  private:
    int capacity;
    uint8_t *data;
    int deviceAddress;

    bool        biosEnabled;
    std::string biosFile;
    uint32_t    biosAddress;
    uint8_t    *biosData;
};

}; // namespace

#endif


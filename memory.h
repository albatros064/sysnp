#ifndef SYSNP_MEMORY_H
#define SYSNP_MEMORY_H

#include "nbus.h"

namespace sysnp {

class Memory : public NBusDevice {
  public:
    Memory();
    virtual ~Memory() {}

    // Device::init()
    virtual void init(Machine &, const libconfig::Setting &);
    // Device::postInit()
    virtual void postInit(Machine &);

    // NBusDevice::clock()
    virtual void clock(NBus &);

  private:
};

}; // namespace

#endif


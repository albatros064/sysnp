#ifndef SYSNP_DEVICE_H
#define SYSNP_DEVICE_H

#include <libconfig.h++>
#include "machine.h"

namespace sysnp {

class Machine;

class Device {
  public:
    virtual void init(Machine&, const libconfig::Setting &) = 0;
    virtual void postInit(Machine&) = 0;
};

}; // namespace
#endif


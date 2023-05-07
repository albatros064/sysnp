#ifndef SYSNP_DEVICE_H
#define SYSNP_DEVICE_H

#include <libconfig.h++>
#include <string>
#include <memory>
#include "machine.h"

namespace sysnp {

class Machine;

class Device {
  public:
    virtual void init(std::shared_ptr<Machine>, const libconfig::Setting &) =0;
    virtual void postInit() =0;

    virtual void clockUp  () =0;
    virtual void clockDown() =0;

    virtual std::string output(uint8_t) =0;
};

}; // namespace
#endif


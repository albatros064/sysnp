#ifndef SYSNP_NBUS_H
#define SYSNP_NBUS_H

#include <string>
#include <array>
#include <vector>

#include "device.h"

namespace sysnp {

using std::string;

class NBus;

class NBusDevice : public Device {
  public:
    virtual void clock(NBus&) = 0;

  protected:
    uint32_t deviceAddress;
};

class NBus : public Device {
  public:
    virtual void init(Machine &, const libconfig::Setting &);
    virtual void postInit(Machine &);

    void     drive(uint8_t,uint32_t);
    uint32_t sense(uint8_t);

    void clock();

  private:
    std::array<uint32_t,10> signals;
    std::vector<NBusDevice *> devices;

    std::vector<string> deviceNames;
    uint32_t deviceAddress;
    uint32_t clockFrequency;
    
};

}; // namespace sysnp
#endif


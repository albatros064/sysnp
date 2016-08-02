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
    NBusDevice() {}
    virtual ~NBusDevice() {}
    virtual void clock(NBus&) = 0;

  protected:
    uint32_t deviceAddress;
};

class NBus : public Device {
  public:
    NBus();
    virtual ~NBus() {};
    virtual void init(Machine *, const libconfig::Setting &);
    virtual void postInit();

    enum Signal { Address, Data, WriteEnable, ReadEnable, InterruptA, InterruptB, InterruptC, InterruptD };

    uint32_t busAddress(bool =false, uint32_t =0); // only 24 bits actual address
    uint16_t busData   (bool =false, uint16_t =0);
    bool     busWrite  (bool =false, bool =false);
    bool     busRead   (bool =false, bool =false);

    void clock();

  private:
    std::array<uint32_t,Signal::InterruptD + 1> signals;
    std::array<uint32_t,Signal::InterruptD + 1> nextSignals;
    std::vector<NBusDevice *> devices;

    std::vector<string> deviceNames;
    uint32_t deviceAddress;
    uint32_t clockFrequency;

    void drive(Signal, uint32_t);
    uint32_t sense(Signal);
};

}; // namespace sysnp
#endif


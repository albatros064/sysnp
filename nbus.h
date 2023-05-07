#ifndef SYSNP_NBUS_H
#define SYSNP_NBUS_H

#include <string>
#include <memory>
#include <array>
#include <vector>

#include "device.h"

namespace sysnp {

using std::string;

enum NBusSignal {
    Address,
    Data,
    WriteEnable,
    ReadEnable,
    Interrupt0,
    Interrupt1,
    Interrupt2,
    Interrupt3,
    NotReady,
};

class NBusInterface;

class NBus : public Device , public std::enable_shared_from_this<NBus>  {
    public:
        NBus();
        virtual ~NBus() {}

        virtual void clockUp  ();
        virtual void clockDown();

        uint32_t sense(NBusSignal);

        void addInterface(std::shared_ptr<NBusInterface>);
        std::shared_ptr<NBusInterface> getIndependentInterface();

        virtual void init(std::shared_ptr<Machine>, const libconfig::Setting &);
        virtual void postInit();
        virtual std::string output(uint8_t);
    private:
        std::vector<std::shared_ptr<NBusInterface>> interfaces;
        std::vector<std::string> deviceNames;
        std::array<uint32_t, NBusSignal::NotReady + 1> signalMasks;

        std::shared_ptr<NBusInterface> selfInterface;

        std::shared_ptr<Machine> machine;
};

class NBusInterface {
    public:
        NBusInterface(std::shared_ptr<NBus>, std::shared_ptr<Device>);
        virtual ~NBusInterface() {};
        void assert(NBusSignal, uint32_t);
        void deassert(NBusSignal);
        uint32_t sense(NBusSignal);

        virtual void clockUp  ();
        virtual void clockDown();
    private:
        std::array<uint32_t, NBusSignal::NotReady + 1> signals;
        std::shared_ptr<NBus> bus;
        std::shared_ptr<Device> device;
        friend uint32_t NBus::sense(NBusSignal);
};

class NBusDevice : public Device {
    public:
        void setInterface(std::shared_ptr<NBusInterface> newInterface) { this->interface = newInterface; }
    protected:
        std::shared_ptr<NBusInterface> interface;
};

}; // namespace sysnp
#endif


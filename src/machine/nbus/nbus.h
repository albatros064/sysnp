#ifndef SYSNP_NBUS_H
#define SYSNP_NBUS_H

#include <string>
#include <sstream>
#include <memory>
#include <array>
#include <vector>

#include "../device.h"

namespace sysnp {

namespace nbus {

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

enum BusPhase {
    BusIdle,
    BusBegin,
    BusWait,
    BusActive,
    BusCleanup
};

class NBusInterface;

class NBus : public Device , public std::enable_shared_from_this<NBus>  {
    public:
        NBus();
        virtual ~NBus() {}

        virtual void clockUp  ();
        virtual void clockDown();

        uint32_t senseSignal(NBusSignal);

        void addInterface(std::shared_ptr<NBusInterface>);
        std::shared_ptr<NBusInterface> getIndependentInterface();

        virtual void init(const libconfig::Setting &);
        virtual void postInit();

        virtual std::string command(std::stringstream&);
    private:
        std::vector<std::shared_ptr<NBusInterface>> interfaces;
        std::vector<std::string> deviceNames;
        std::array<uint32_t, NBusSignal::NotReady + 1> signalMasks;

        std::shared_ptr<NBusInterface> selfInterface;
};

class NBusInterface {
    public:
        NBusInterface(std::shared_ptr<NBus>, std::shared_ptr<Device>);
        virtual ~NBusInterface() {};
        void assertSignal(NBusSignal, uint32_t);
        void deassertSignal(NBusSignal);
        uint32_t senseSignal(NBusSignal);

        virtual void clockUp  ();
        virtual void clockDown();
    private:
        std::array<uint32_t, NBusSignal::NotReady + 1> signals;
        std::shared_ptr<NBus> bus;
        std::shared_ptr<Device> device;
        friend uint32_t NBus::senseSignal(NBusSignal);
};

class NBusDevice : public Device {
    public:
        void setInterface(std::shared_ptr<NBusInterface> newInterface) { this->interface = newInterface; }
    protected:
        std::shared_ptr<NBusInterface> interface;
};

}; // namespace nbus

}; // namespace sysnp
#endif


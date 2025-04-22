#ifndef SYSNP_DEVICE_H
#define SYSNP_DEVICE_H

#include <string>
#include <memory>
#include "machine.h"

namespace sysnp {

class Machine;

class Device {
    public:
        virtual void init(ryml::NodeRef &) =0;
        virtual void postInit() =0;

        virtual void clockUp  () =0;
        virtual void clockDown() =0;

        virtual std::string command(std::stringstream&) =0;

        void setMachine(std::shared_ptr<Machine> machine) {
            this->machine = machine;
        }
    protected:
        std::shared_ptr<Machine> machine;
};

}; // namespace
#endif


#ifndef SYSNP_N16R_H
#define SYSNP_N16R_H

#include "../nbus.h"

namespace sysnp {

namespace nbus {

namespace n16r {

class N16R : public NBusDevice {
    public:
        N16R();
        virtual ~N16R();

        virtual void init(const libconfig::Setting &);
        virtual void postInit();

        virtual void clockUp();
        virtual void clockDown();

        virtual std::string command(std::stringstream&);
    private:

        uint16_t registerFile[16];
};

}; // namespace n16r

}; // namespace nbus

}; // namespace sysnp

#endif

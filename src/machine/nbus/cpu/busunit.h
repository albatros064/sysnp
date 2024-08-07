#ifndef SYSNP_N16R_BUS_H
#define SYSNP_N16R_BUS_H

#include "../nbus.h"

namespace sysnp {

namespace nbus {

namespace n16r {

struct BusOperation {
    uint32_t address = 0;
    bool isRead = true;
    std::vector<uint16_t> data;
    int bytes = 0;
    bool isValid = true;
};

class BusUnit {
    public:
        void setBusInterface(std::shared_ptr<NBusInterface>);
        void reset();
        void clockUp();
        void clockDown();

        bool isIdle();
        void queueOperation(BusOperation);
        bool hasData();
        uint16_t getWord();

        uint8_t hasInterrupt();

    private:
        std::shared_ptr<NBusInterface> interface;

        BusPhase phase;
        bool notReady;
        int addressCounter;
        int dataCounter;
        int readMode;
        int writeMode;

        BusOperation currentOperation;

        uint8_t interruptState;
};

}; // namespace n16r

}; // namespace nbus

}; // namespace sysnp

#endif

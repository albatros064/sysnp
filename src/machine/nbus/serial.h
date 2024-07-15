#ifndef SYSNP_SERIAL_H
#define SYSNP_SERIAL_H

#include <string>
#include <sstream>
#include <mutex>
#include <thread>

#include "nbus.h"

namespace sysnp {

namespace nbus {

class Serial : public NBusDevice {
    public:
        Serial() {}
        virtual ~Serial();

        virtual void init(const libconfig::Setting &);
        virtual void postInit();

        virtual void clockUp();
        virtual void clockDown();

        virtual std::string command(std::stringstream &);
    private:
        uint32_t ioAddress;
        BusPhase phase;
        std::string ttyFile;

        NBusSignal interrupt;

        int holdup;

        uint32_t dataLatch;
        uint32_t addressLatch;
        uint32_t readLatch;
        uint32_t writeLatch;

        std::mutex outDataMutex;
        std::mutex inDataMutex;
        uint8_t outData;
        uint8_t inData;

        bool hasOutData;
        bool hasInData;
        bool lastOutData;

        bool        ttyRunning;
        std::thread ttyReadThread;
        std::thread ttyWriteThread;
        int         ttyHandle;

        void ttyRead();
        void ttyWrite();

        friend void ttyExecute(Serial&, bool);
};

void ttyExecute(Serial&, bool);


}; // namespace nbus
}; // namespace sysnp

#endif

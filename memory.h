#ifndef SYSNP_MEMORY_H
#define SYSNP_MEMORY_H

#include <string>

#include "nbus.h"

namespace sysnp {

enum MemoryStatus {
    Ready,
    ReadLatency,
    WriteLatency,
    Cleanup
};

class MemoryModule;

class Memory : public NBusDevice {
  public:
    Memory();
    virtual ~Memory();

    virtual void init(std::shared_ptr<Machine>, const libconfig::Setting &);
    virtual void postInit();

    virtual void clockUp();
    virtual void clockDown();

    virtual std::string output(uint8_t);

  private:
    std::shared_ptr<Machine> machine;
    std::vector<std::shared_ptr<MemoryModule>> modules;
    std::shared_ptr<MemoryModule> selectedModule;

    uint32_t ioHoleAddress;
    uint32_t ioHoleSize;

    MemoryStatus status;
    uint8_t latency;

    uint32_t dataLatch;
    uint32_t addressLatch;
    uint32_t readLatch;
    uint32_t writeLatch;
};

class MemoryModule {
    public:
        MemoryModule(uint32_t, uint32_t, bool, std::string, uint8_t, uint8_t);
        ~MemoryModule();

        bool containsAddress(uint32_t);
        uint8_t getReadLatency();
        uint8_t getWriteLatency();

        uint8_t read(uint32_t);
        void write(uint32_t, uint8_t);
    private:
        uint32_t startAddress;
        uint32_t size;
        uint8_t readLatency;
        uint8_t writeLatency;
        uint8_t *data;
        bool rom;
};

}; // namespace

#endif


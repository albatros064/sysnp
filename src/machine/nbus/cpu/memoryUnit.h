#ifndef SYSNP_MEMORYUNIT_H
#define SYSNP_MEMORYUNIT_H

#include <cstdint>
#include <vector>
#include <set>
#include "busunit.h"
#include "cache.h"

namespace sysnp {

namespace nbus {

namespace n16r {

enum CacheType {
    InstructionCache,
    DataCache,
    UnifiedL2Cache
};

enum MemorySegment {
    MemorySegmentU0,
    MemorySegmentK0,
    MemorySegmentK1,
    MemorySegmentK2
};

enum CacheMode {
    WriteThroughCache,
    WriteBackCache
};

enum MemoryReadType {
    InstructionRead,
    DataRead
};

enum MemoryOpType {
    MemoryOpInstructionRead,
    MemoryOpDataRead,
    MemoryOpDataWrite
};

enum MemoryCheckResult {
    MemoryCheckContainsNone,
    MemoryCheckContainsPartial,
    MemoryCheckContainsSingle,
    MemoryCheckContainsSplit,
    MemoryCheckNoWrite,
    MemoryCheckNoRead,
    MemoryCheckNoExecute,
    MemoryCheckNoPresent
};

struct MemoryCheck {
    MemoryCheck() {}
    MemoryCheck(MemoryCheckResult _result):result(_result) {}
    MemoryCheck(CacheCheck);
    ~MemoryCheck() {}

    MemoryCheckResult result;

    bool isException() { return result >= MemoryCheckNoWrite; }
    bool isComplete() { return result == MemoryCheckContainsSingle || result == MemoryCheckContainsSplit; }
    bool isMiss() { return result < MemoryCheckContainsSingle; }
};

struct MemoryOperation {
    uint16_t operationId;

    uint32_t address;
    uint32_t asid;
    std::vector<uint8_t> data;
    int bytes = 0;

    MemoryOpType type;

    bool committed = false;

    bool isValid() { return bytes > 0; }
    bool isReady() { return bytes > 0 && committed; }
    void invalidate() { bytes = 0; committed = false; }

    BusOperation getBusOperation();
    CacheCheck contains(CacheType, uint32_t, int, uint32_t);

    const static uint16_t invalidOperationId = 0xffff;
};

class MemoryUnit {
    public:
        MemoryUnit();
        ~MemoryUnit() {}

        void setCache(CacheType, int, int, int, int);
        void addNoCacheRegion(uint32_t, uint32_t);

        MemoryCheck check(MemoryOpType, uint32_t, int, uint32_t);
        uint32_t read(CacheType, uint32_t, int, uint32_t);

        uint16_t queueRead(MemoryReadType, uint32_t, int, uint32_t);

        uint16_t queueOperation     (MemoryOperation);
        void     commitOperation    (uint16_t);
        void     invalidateOperation(uint16_t);

        bool isOperationPrepared();
        MemoryOperation getOperation();
        BusOperation getBusOperation();
        void ingestWord(uint16_t);

        MemorySegment getSegment(uint32_t);

        std::string describeQueuedOperations();
        std::string listContents(std::stringstream &);
    private:
        std::vector<MemoryOperation> queuedOperations;
        MemoryOperation pendingOperation;
        MemoryOperation lastUncachedRead;
        std::map<CacheType, Cache<uint32_t, uint8_t, uint32_t, uint8_t>> caches;
        Cache<uint32_t, uint16_t, uint32_t, uint16_t> tlb;

        std::vector<std::pair<uint32_t, uint32_t>> noCacheRegions;

        bool canCache(uint32_t, uint32_t);

        uint16_t isReadQueued(MemoryReadType, uint32_t, uint32_t);

        void applyWrite(MemoryOperation);

        uint32_t convert(std::vector<uint8_t>);
};

}; // namespace n16r

}; // namespace nbus

}; // namespace sysnp

#endif

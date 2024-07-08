#ifndef SYSNP_CACHE_H
#define SYSNP_CACHE_H

#include <vector>
#include <set>
#include <map>

#include "busunit.h"

namespace sysnp {

namespace nbus {

namespace n16r {

class CacheLine {
    public:
        CacheLine();
        CacheLine(int, int, int);
        virtual ~CacheLine() {}

        bool contains(uint32_t, uint32_t);
        std::vector<uint8_t> read(uint32_t, uint32_t, int);
        uint32_t getFlags();

        void load(uint32_t, uint32_t, uint32_t, std::vector<uint8_t>);
        void write(uint32_t, uint32_t, std::vector<uint8_t>);

        void invalidate();

        bool valid;

        uint8_t lru;
    private:
        uint32_t tag;
        uint32_t asid;
        uint32_t flags;
        std::vector<uint8_t> contents;

        uint32_t tagMask;
        uint32_t lineMask;
};

class CacheBin {
    public:
        CacheBin();
        CacheBin(int, int, int, int);
        virtual ~CacheBin() {}

        bool contains(uint32_t, uint32_t);
        std::vector<uint8_t> read(uint32_t, uint32_t, int);

        void invalidate(uint32_t, uint32_t);

        void load(uint32_t, uint32_t, uint32_t, std::vector<uint8_t>);
        void write(uint32_t, uint32_t, std::vector<uint8_t>);
    private:
        std::vector<CacheLine> ways;
};

class Cache {
    public:
        Cache();
        Cache(int, int, int, int);
        virtual ~Cache() {}

        bool contains(uint32_t, uint32_t);
        uint8_t  readByte (uint32_t, uint32_t);
        uint16_t readWord (uint32_t, uint32_t);
        uint32_t readDword(uint32_t, uint32_t);

        //uint32_t read(uint32_t, uint32_t, int);

        void invalidate(uint32_t, uint32_t);

        void load(uint32_t, uint32_t, uint32_t, std::vector<uint8_t>);
        void write(uint32_t, uint32_t, std::vector<uint8_t>);

        uint32_t getLineMask();
        int      getLineBytes();
    private:
        int addressBits;
        int binBits;
        int lineBits;

        uint32_t binMask;
        uint32_t lineMask;

        std::vector<CacheBin> bins;

        uint32_t read(uint32_t, uint32_t, int);
};

enum CacheType {
    InstructionCache,
    DataCache,
    UnifiedL2Cache
};

enum CacheMode {
    WriteThroughCache,
    WriteBackCache
};

enum MemoryReadType {
    InstructionRead,
    DataRead
};

enum MemoryOperationType {
    MemoryOperationInstructionRead,
    MemoryOperationDataRead,
    MemoryOperationDataWrite
};

struct MemoryOperation {
    uint16_t operationId;

    uint32_t address;
    uint32_t asid;
    std::vector<uint8_t> data;
    int bytes = 0;

    MemoryOperationType type;

    bool committed = false;

    bool isValid() { return bytes > 0; }
    bool isReady() { return bytes > 0 && committed; }
    void invalidate() { bytes = 0; committed = false; }

    BusOperation asBusOperation();

    const static uint16_t invalidOperationId = 0xffff;
};

class CacheController {
    public:
        void setCache(CacheType, int, int, int, int);
        void addNoCacheRegion(uint32_t, uint32_t);

        //void setCacheMode(CacheMode);

        bool contains(CacheType, uint32_t, uint32_t);
        uint16_t read(CacheType, uint32_t, uint32_t);

        uint16_t queueRead(MemoryReadType, uint32_t, uint32_t);

        uint16_t queueOperation     (MemoryOperation);
        void     commitOperation    (uint16_t);
        void     invalidateOperation(uint16_t);

        bool isOperationPrepared();
        MemoryOperation getOperation();
        void ingestWord(uint16_t);

        std::string describeQueuedOperations();
        std::string listContents(std::stringstream &);
    private:
        std::vector<MemoryOperation> queuedOperations;
        MemoryOperation pendingOperation;
        MemoryOperation lastUncachedRead;
        std::map<CacheType, Cache> caches;
        //CacheMode cacheMode;

        std::vector<std::pair<uint32_t, uint32_t>> noCacheRegions;

        bool canCache(uint32_t);

        uint16_t isReadQueued(MemoryReadType, uint32_t, uint32_t);
};

}; // namespace n16r

}; // namespace nbus

}; // namespace sysnp


#endif

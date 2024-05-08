#include "cache.h"
#include <iostream>
#include <iomanip>

namespace sysnp {

namespace nbus {

namespace n16r {

CacheLine::CacheLine():valid(false), lru(0) {}
CacheLine::CacheLine(int _addressBits, int _tagBits, int _lineBits):valid(false), lru(0), contents(1 << _lineBits) {
    tagMask = lineMask = 0xffffffff;
    tagMask  <<= (_addressBits - _tagBits );
    lineMask >>= (_addressBits - _lineBits);
}

bool CacheLine::contains(uint32_t address, uint32_t asid) {
    return valid && this->asid == asid && (address & tagMask) == tag;
}

std::vector<uint8_t> CacheLine::read(uint32_t address, uint32_t asid, int count) {
    std::vector<uint8_t> bytes(count);

    address &= lineMask;
    for (int b = 0; b < count; b++) {
        bytes[b] = contents[address + b];
    }

    return bytes;
}

void CacheLine::invalidate() {
    valid = false;
    lru = 0;
}

void CacheLine::load(uint32_t _address, uint32_t _asid, uint32_t _flags, std::vector<uint8_t> _contents) {
    tag   = _address & tagMask;
    asid  = _asid;
    flags = _flags;
    valid = true;
    contents = _contents;
}

void CacheLine::write(uint32_t address, uint32_t asid, std::vector<uint8_t> data) {
    uint32_t offset = address & lineMask;
    for (int i = 0; i < data.size(); i++) {
        contents[offset + i] = data[i];
    }
}

CacheBin::CacheBin(int _addressWidth, int _tagWidth, int _lineBits, int _ways) {
    for (int w = 0; w < _ways; w++) {
        CacheLine way(_addressWidth, _tagWidth, _lineBits);
        ways.push_back(way);
    }
}

bool CacheBin::contains(uint32_t address, uint32_t asid) {
    for (CacheLine& way: ways) {
        if (way.contains(address, asid)) {
            return true;
        }
    }

    return false;
}

std::vector<uint8_t> CacheBin::read(uint32_t address, uint32_t asid, int count) {
    for (CacheLine& way: ways) {
        if (way.contains(address, asid)) {
            auto previousLru = way.lru;

            for (CacheLine& wayLru: ways) {
                if (wayLru.lru >= previousLru && wayLru.lru > 0) {
                    wayLru.lru--;
                }
            }

            way.lru = ways.size() - 1;
            return way.read(address, asid, count);
        }
    }
    std::vector<uint8_t> empty;
    return empty;
}

void CacheBin::invalidate(uint32_t address, uint32_t asid) {
    for (CacheLine& way: ways) {
        if (way.contains(address, asid)) {
            way.invalidate();
            break;
        }
    }
}

void CacheBin::load(uint32_t address, uint32_t asid, uint32_t flags, std::vector<uint8_t> contents) {
    // first, try to evict an invalid way
    for (CacheLine& way: ways) {
        if (!way.valid) {
            way.load(address, asid, flags, contents);
            return;
        }
    }
    // then, try to evict an old way
    for (CacheLine& way: ways) {
        if (way.lru == 0) {
            way.load(address, asid, flags, contents);
            return;
        }
    }
}

void CacheBin::write(uint32_t address, uint32_t asid, std::vector<uint8_t> contents) {
    for (CacheLine& way: ways) {
        if (way.contains(address, asid)) {
            way.write(address, asid, contents);
            break;
        }
    }
}

Cache::Cache(){}

Cache::Cache(int _addressBits, int _binBits, int _lineBits, int _ways):
        addressBits(_addressBits), binBits(_binBits), lineBits(_lineBits) {
    int tagBits = _addressBits - _binBits - _lineBits;
    int binCount = 1 << _binBits;
    for (int b = 0; b < binCount; b++) {
        CacheBin bin(_addressBits, tagBits, _lineBits, _ways);
        bins.push_back(bin);
    }
    lineMask = 0xffffffff;
    lineMask >>= (32 - lineBits);
    binMask = 0xffffffff;
    binMask >>= tagBits;
    binMask ^= lineMask;
}

bool Cache::contains(uint32_t address, uint32_t asid) {
    int targetBin = (address & binMask) >> lineBits;
    return bins[targetBin].contains(address, asid);
}

uint8_t Cache::readByte(uint32_t address, uint32_t asid) {
    return (uint8_t) (read(address, asid, 1) & 0xff);
}
uint16_t Cache::readWord(uint32_t address, uint32_t asid) {
    if (address & 1) {
        // misaligned
    }
    return (uint16_t) (read(address, asid, 2) & 0xffff);
}
uint32_t Cache::readDword(uint32_t address, uint32_t asid) {
    if (address & 3) {
        // misaligned
    }
    return read(address, asid, 4);
}

uint32_t Cache::read(uint32_t address, uint32_t asid, int count) {
    int targetBin = (address & binMask) >> lineBits;
    auto byteValues = bins[targetBin].read(address, asid, count);

    uint32_t value = 0;

    for (auto byte: byteValues) {
        value <<= 8;
        value |= byte;
    }

    return value;
}

void Cache::invalidate(uint32_t address, uint32_t asid) {
    int targetBin = (address & binMask) >> lineBits;
    bins[targetBin].invalidate(address, asid);
}

void Cache::load(uint32_t startAddress, uint32_t asid, uint32_t flags, std::vector<uint8_t> contents) {
    int targetBin = (startAddress & binMask) >> lineBits;
    bins[targetBin].load(startAddress, asid, flags, contents);
}

void Cache::write(uint32_t startAddress, uint32_t asid, std::vector<uint8_t> contents) {
    int targetBin = (startAddress & binMask) >> lineBits;
    bins[targetBin].write(startAddress, asid, contents);
}

uint32_t Cache::getLineMask() {
    return lineMask;
}
int Cache::getLineBytes() {
    return 1 << lineBits;
}

BusOperation MemoryOperation::asBusOperation() {
    BusOperation op;
    op.address = address;
    op.isRead = type != MemoryOperationDataWrite;
    op.bytes = bytes;

    if (data.size() > 0) {
        if (bytes == 1) {
            if (address & 1) {
                op.data.push_back(data[0] << 8);
            }
            else {
                op.data.push_back(data[0]);
            }
        }
        else {
            for (int i = 0; i < data.size(); i += 2) {
                uint16_t word = op.data[i];
                word |= ((uint16_t) op.data[i + 1]) << 8;
                op.data.push_back(word);
            }
        }
    }

    return op;
}

void CacheController::setCache(CacheType type, int _addressBits, int _binBits, int _lineBits, int _ways) {
    caches.emplace(std::piecewise_construct, std::forward_as_tuple(type), std::forward_as_tuple(_addressBits, _binBits, _lineBits, _ways));
}

void CacheController::addNoCacheRegion(uint32_t start, uint32_t length) {
    std::pair<uint32_t, uint32_t> region(start, length);
    noCacheRegions.push_back(region);
}

bool CacheController::contains(CacheType type, uint32_t address, uint32_t asid) {
    return canCache(address) &&
        caches.contains(type) &&
        caches[type].contains(address, asid);
}

uint16_t CacheController::read(CacheType type, uint32_t address, uint32_t asid) {
    if (caches[type].contains(address, asid)) {
        return caches[type].readWord(address, asid);
    }
    return 0;
}

uint16_t CacheController::queueRead(MemoryReadType type, uint32_t address, uint32_t asid) {
    auto alreadyQueued = isReadQueued(type, address, asid);
    if (alreadyQueued) {
        return alreadyQueued;
    }

    int bytes = 2;
    CacheType cacheType = type == InstructionRead ? InstructionCache : DataCache;
    if (canCache(address) && caches.contains(cacheType)) {
        address = address & ~caches[cacheType].getLineMask();
        bytes   =            caches[cacheType].getLineBytes();
    }

    MemoryOperation operation;
    operation.address = address;
    operation.asid    = asid;
    operation.bytes   = bytes;
    operation.type    = type == InstructionRead ? MemoryOperationInstructionRead : MemoryOperationDataRead;
    operation.committed = true;

    return queueOperation(operation);
}

uint16_t CacheController::queueOperation(MemoryOperation operation) {
    std::set<uint16_t> existingIds;
    uint16_t operationId = 0;
    for (auto op: queuedOperations) {
        existingIds.insert(op.operationId);
        operationId = op.operationId;
    }

    do {
        operationId++;
    }
    while (existingIds.contains(operationId));

    operation.operationId = operationId;

    queuedOperations.insert(queuedOperations.begin(), operation);

    return operationId;
}
void CacheController::commitOperation(uint16_t operationId) {
    for (MemoryOperation& op: queuedOperations) {
        if (op.operationId == operationId && op.isValid()) {
            op.committed = true;
            if (op.type == MemoryOperationDataWrite) {
                for (auto c: {InstructionCache, DataCache, UnifiedL2Cache}) {
                    if (contains(c, op.address, op.asid)) {
                        caches[c].write(op.address, op.asid, op.data);
                    }
                }
            }
            break;
        }
    }
}
void CacheController::invalidateOperation(uint16_t operationId) {
    for (auto iter = queuedOperations.begin(); iter != queuedOperations.end(); iter++) {
        if (iter->operationId == operationId) {
            queuedOperations.erase(iter);
            break;
        }
    }
}

bool CacheController::isOperationPrepared() {
    if (pendingOperation.isValid()) {
        return false;
    }

    for (auto op: queuedOperations) {
        if (op.committed) {
            return true;
        }
    }

    return false;
}
MemoryOperation CacheController::getOperation() {
    for (int i = queuedOperations.size() - 1; i >= 0; --i) {
        if (queuedOperations[i].isReady()) {
            auto op = queuedOperations[i];
            queuedOperations.erase(queuedOperations.begin() + i);
            if (op.type != MemoryOperationDataWrite) {
                pendingOperation = op;
            }
            return op;
        }
    }

    MemoryOperation dummy;
    return dummy;
}

void CacheController::ingestWord(uint16_t word) {
    if (!pendingOperation.isValid()) {
        // should we error? not sure.
        return;
    }

    uint8_t low = word & 0xff;
    uint8_t high = word >> 8;

    MemoryOperation& pending = pendingOperation;

    pending.data.push_back(high);
    pending.data.push_back(low);
    if (pending.data.size() == pending.bytes) {
        CacheType type = pending.type == MemoryOperationInstructionRead ? InstructionCache : DataCache;
        caches[type].load(pending.address, pending.asid, 0, pending.data);

        pending.invalidate();
    }
}

bool CacheController::canCache(uint32_t address) {
    for (auto region: noCacheRegions) {
        if (region.first <= address && (region.first + region.second) >= address) {
            return false;
        }
    }

    return true;
}

uint16_t CacheController::isReadQueued(MemoryReadType type, uint32_t address, uint32_t asid) {
    MemoryOperationType opType = type == InstructionRead ? MemoryOperationInstructionRead : MemoryOperationDataRead;
    for (auto op: queuedOperations) {
        if (op.type != opType || op.asid != asid) {
            continue;
        }
        if (op.address <= address && (op.address + op.bytes) >= address) {
            return op.operationId;
        }
    }

    if (pendingOperation.type == opType &&
        pendingOperation.asid == asid &&
        pendingOperation.address <= address &&
        (pendingOperation.address + pendingOperation.bytes) > address) {
        return pendingOperation.operationId;
    }

    return 0;
}


}; // namespace n16r

}; // namespace nbus

}; // namespace sysnp

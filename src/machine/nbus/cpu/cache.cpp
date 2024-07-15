#include "cache.h"
#include <iostream>
#include <iomanip>
#include <iterator>

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

std::vector<uint8_t> CacheLine::read(uint32_t address, int count, uint32_t asid) {
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

std::vector<uint8_t> CacheBin::read(uint32_t address, int count, uint32_t asid) {
    for (CacheLine& way: ways) {
        if (way.contains(address, asid)) {
            auto previousLru = way.lru;

            for (CacheLine& wayLru: ways) {
                if (wayLru.lru >= previousLru && wayLru.lru > 0) {
                    wayLru.lru--;
                }
            }

            way.lru = ways.size() - 1;
            return way.read(address, count, asid);
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

CacheCheck Cache::contains(uint32_t address, int count, uint32_t asid) {
    int startBin = (address & binMask) >> lineBits;
    int endBin = ((address + count - 1) & binMask) >> lineBits;

    bool startContains = bins[startBin].contains(address, asid);

    if (startBin != endBin) {
        bool endContains = bins[endBin].contains(address + count, asid);
        if (startContains && endContains) {
            return CacheContainsSplit;
        }
        else if (startContains || endContains) {
            return CacheContainsPartial;
        }
    }
    else if (startContains) {
        return CacheContainsSingle;
    }

    return CacheContainsNone;
}

uint8_t Cache::readByte(uint32_t address, uint32_t asid) {
    return (uint8_t) (read(address, 1, asid) & 0xff);
}
uint16_t Cache::readWord(uint32_t address, uint32_t asid) {
    if (address & 1) {
        // misaligned
    }
    address &= 0xfffffffe;
    return (uint16_t) (read(address, 2, asid) & 0xffff);
}
uint32_t Cache::readDword(uint32_t address, uint32_t asid) {
    if (address & 3) {
        // misaligned
    }
    return read(address, 4, asid);
}

uint32_t Cache::read(uint32_t address, int count, uint32_t asid) {
    int startBin = (address & binMask) >> lineBits;
    int endBin =  ((address + count - 1) & binMask) >> lineBits;

    int startCount = count;
    int endCount = 0;


    if (startBin != endBin) {
        uint32_t endAddress = ((address + count) >> lineBits) << lineBits;
        startCount = endAddress - address;
        endCount = count - startCount;
    }

    auto byteValues = bins[startBin].read(address, startCount, asid);
    if (endCount > 0) {
        auto endValues = bins[endBin].read(address + startCount, endCount, asid);
        for (auto endVal: endValues) {
            byteValues.push_back(endVal);
        }
    }

    uint32_t value = 0;

    for (auto iter = byteValues.crbegin(); iter != byteValues.crend(); iter++) {
        value <<= 8;
        value |= *iter;
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

void Cache::write(uint32_t address, uint32_t asid, std::vector<uint8_t> contents) {
    int count = contents.size();

    int startBin = (address & binMask) >> lineBits;
    int endBin =  ((address + count - 1) & binMask) >> lineBits;

    if (startBin != endBin) {
        count = (((address + count) >> lineBits) << lineBits) - address;
        std::vector<uint8_t> endContents(std::next(contents.begin(), count), contents.end());
        contents.resize(count);

        bins[endBin].write(endBin, asid, endContents);
    }

    bins[startBin].write(address, asid, contents);
}

uint32_t Cache::getLineMask() {
    return lineMask;
}
int Cache::getLineBytes() {
    return 1 << lineBits;
}

BusOperation MemoryOperation::getBusOperation() {
    BusOperation op;
    op.address = address;
    op.isValid = true;
    op.isRead = type != MemoryOperationDataWrite;
    op.bytes = bytes;

    bool hasData = data.size() > 0;

    uint16_t word;

    if ((address & 1) || (bytes & 1)) {
        // we're misaligned somehow. split into multiple
        // bus operations
        if (address & 1) {
            // we are byte aligned. chip off one
            // byte and send it
            op.bytes = 1;
            address = address & 0xfffffffe;

            if (hasData) {
                word = ((uint16_t) data.front()) << 8;
            }
        }
        else {
            // we are word aligned.
            op.bytes = bytes > 1 ? 2 : 1;
            if (hasData) {
                word = data.front();
            }

            if (op.bytes == 2) {
                if (hasData) {
                    data.erase(data.begin());
                    word |= ((uint16_t) data.front()) << 8;
                }
                address += 2;
            }
        }
        bytes -= op.bytes;

        if (hasData) {
            data.erase(data.begin());
            op.data.push_back(word);
        }
    }
    else {
        // we're aligned and a multiple of 2.
        // just word-ify it, and send it on its way
        if (hasData) {
            for (int i = 0; i < data.size(); i += 2) {
                word = data[i];
                word |= ((uint16_t) data[i + 1]) << 8;
                op.data.push_back(word);
            }
        }

        bytes = 0;
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

CacheCheck CacheController::contains(CacheType type, uint32_t address, int count, uint32_t asid) {
    if (!canCache(address)) {
        if (lastUncachedRead.isValid() && lastUncachedRead.address == (lastUncachedRead.address & address)) {
            return CacheContainsSingle;
        }
        return CacheContainsNone;
    }

    if (!caches.contains(type)) {
        return CacheContainsNone;
    }

    return caches[type].contains(address, count, asid);
}

uint32_t CacheController::read(CacheType type, uint32_t address, int count, uint32_t asid) {
    if (lastUncachedRead.isValid() && lastUncachedRead.address == (lastUncachedRead.address & address)) {
        uint32_t value = 0;
        for (auto iter = lastUncachedRead.data.crbegin(); iter != lastUncachedRead.data.crend(); iter++) {
            value <<= 8;
            value |= *iter;
        }

        lastUncachedRead.invalidate();
        return value;
    }

    if (caches[type].contains(address, count, asid) != CacheContainsNone) {
        return caches[type].read(address, count, asid);
    }
    return 0;
}

uint16_t CacheController::queueRead(MemoryReadType type, uint32_t address, int count, uint32_t asid) {
    auto alreadyQueued = isReadQueued(type, address, asid);
    if (alreadyQueued) {
        return alreadyQueued;
    }

    int bytes = count;
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
    int queuedCount = 0;
    for (auto op: queuedOperations) {
        existingIds.insert(op.operationId);
        operationId = op.operationId;

        if (operation.type == op.type && op.isValid()) {
            queuedCount++;
            if (queuedCount > 1) {
                return MemoryOperation::invalidOperationId;
            }
        }
    }

    do {
        operationId++;
    }
    while (existingIds.contains(operationId) || operationId == MemoryOperation::invalidOperationId);

    operation.operationId = operationId;

    queuedOperations.insert(queuedOperations.begin(), operation);

    return operationId;
}
void CacheController::commitOperation(uint16_t operationId) {
    for (MemoryOperation& op: queuedOperations) {
        if (op.operationId == operationId && op.isValid()) {
            op.committed = true;
            if (op.type == MemoryOperationDataWrite) {
                applyWrite(op);
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
            if (op.type == MemoryOperationDataWrite) {
                applyWrite(op);
            }
            else {
                pendingOperation = op;
            }
            return op;
        }
    }

    MemoryOperation dummy;
    return dummy;
}

BusOperation CacheController::getBusOperation() {
    for (int i = queuedOperations.size() - 1; i >= 0; --i) {
        if (queuedOperations[i].isReady()) {
            auto op = queuedOperations[i];

            if (op.type == MemoryOperationDataWrite) {
                applyWrite(op);
            }
            else {
                pendingOperation = op;
            }

            BusOperation busOp = op.getBusOperation();

            if (!op.isValid()) {
                queuedOperations.erase(std::next(queuedOperations.begin(), i));
            }
            else {
                queuedOperations[i] = op;
            }
            return busOp;
        }
    }

    BusOperation busOp;
    return busOp;
}

void CacheController::applyWrite(MemoryOperation op) {
    for (auto c: {InstructionCache, DataCache, UnifiedL2Cache}) {
        if (contains(c, op.address, op.data.size(), op.asid) != CacheContainsNone) {
            caches[c].write(op.address, op.asid, op.data);
        }
    }
}

void CacheController::ingestWord(uint16_t word) {
    if (!pendingOperation.isValid()) {
std::cout << "pending invalid" << std::endl;
        // should we error? not sure.
        return;
    }

    uint8_t low = word & 0xff;
    uint8_t high = word >> 8;

    MemoryOperation& pending = pendingOperation;

    pending.data.push_back(low);
    pending.data.push_back(high);

    if (pending.data.size() == pending.bytes) {
        if (canCache(pending.address)) {
            CacheType type = pending.type == MemoryOperationInstructionRead ? InstructionCache : DataCache;
            caches[type].load(pending.address, pending.asid, 0, pending.data);
        }
        else {
            lastUncachedRead = pending;
        }

        pending.invalidate();
    }
}

std::string CacheController::describeQueuedOperations() {
    std::stringstream response;

    response << "C  ADDR      T B   BYTES";

    for (auto iter = queuedOperations.cbegin(); iter != queuedOperations.cend(); iter++) {
        //if (!(iter->isValid())) {
            //continue;
        //}
        response << std::endl << (iter->committed ? 1 : 0) << "  ";
        response << std::setw(8) << std::setfill('0') << std::hex << iter->address << "  ";
        response << iter->type << " " << std::setw(2) << std::dec << iter->bytes << " ";

        for (auto it = iter->data.cbegin(); it != iter->data.cend(); it++) {
            response << " " << std::setw(2) << std::setfill('0') << std::hex << (int)(*it);
        }
    }

    if (pendingOperation.isValid()) {
        response << std::endl << "p  ";
        response << std::setw(8) << std::setfill('0') << std::hex << pendingOperation.address << "  ";
        response << pendingOperation.type << " " << std::setw(2) << std::dec << pendingOperation.bytes << " ";

        for (auto it = pendingOperation.data.cbegin(); it != pendingOperation.data.cend(); it++) {
            response << " " << std::setw(2) << std::setfill('0') << std::hex << (int) (*it);
        }
    }

    return response.str();
}

std::string CacheController::listContents(std::stringstream &input) {
    std::stringstream response;
    std::string locationStr;
    uint16_t length = 64;
    uint32_t location;

    input >> locationStr >> length;
    location = std::stoul(locationStr, nullptr, 0);
    location >>= 3;
    location <<= 3;
    length >>= 1;

    uint32_t asid = 0;

    for (int i = 0; i < length / 4; i++) {
        response << std::setw(8) << std::setfill('0') << std::hex << (location + (i << 3)) << " ";
        for (int b = 0; b < 4; b++) {
            uint32_t address = location + (i << 3) + (b << 1);
            if (contains(DataCache, address, 2, asid)) {
                uint16_t datum = read(DataCache, address, 2, asid);
                response << " " << std::setw(2) << std::setfill('0') << std::hex << (datum & 0xff);
                response << " " << std::setw(2) << std::setfill('0') << std::hex << (datum >> 8);
            }
            else {
                response << " .. ..";
            }
        }
        response << std::endl;
    }
    return response.str();
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

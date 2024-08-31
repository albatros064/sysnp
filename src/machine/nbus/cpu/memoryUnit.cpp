#include "memoryUnit.h"
#include <iostream>
#include <iomanip>
#include <iterator>

namespace sysnp {

namespace nbus {

namespace n16r {

MemoryUnit::MemoryUnit() {}

void MemoryUnit::setCache(CacheType type, int _addressBits, int _binBits, int _lineBits, int _ways) {
    int _wayBits = 0;
    while (_ways > 1) {
        _wayBits++;
        _ways >>= 1;
    }
    caches.emplace(std::piecewise_construct, std::forward_as_tuple(type), std::forward_as_tuple(_binBits, _lineBits, _wayBits, false, -1, 0));
}

void MemoryUnit::addNoCacheRegion(uint32_t start, uint32_t length) {
    std::pair<uint32_t, uint32_t> region(start, length);
    noCacheRegions.push_back(region);
}

MemoryCheck MemoryUnit::check(MemoryOpType type, uint32_t address, int count, uint32_t asid) {
    if (!canCache(address, asid)) {
        if (lastUncachedRead.isValid() && lastUncachedRead.inAddress == (lastUncachedRead.inAddress & address)) {
            return MemoryCheckContainsSingle;
        }
        return MemoryCheckContainsNone;
    }

    CacheType cacheType = type == MemoryOpInstructionRead ? InstructionCache : DataCache;

    auto pendingContains = pendingOperation.contains(cacheType, address, count, asid);
    if (pendingContains != CacheContainsNone) {
        return pendingContains;
    }

    // Check if it's in the cache
    CacheCheck cacheResult = CacheContainsNone;
    if (caches.contains(cacheType)) {
        cacheResult = caches[cacheType].contains(address, count, asid);
    }

    // Cache Hit -- return the hit
    if (cacheResult >= CacheContainsSingle) {
        // Check modes
        auto cacheFlags = caches[cacheType].getFlags(address, asid);
        switch (type) {
            case MemoryOpInstructionRead:
                if (cacheFlags & CACHE_FLAG_NOEXEC) {
                    return MemoryCheckNoExecute;
                }
                else if (cacheFlags & CACHE_FLAG_NOREAD) {
                    return MemoryCheckNoRead;
                }
                break;
            case MemoryOpDataWrite:
                if (!(cacheFlags & CACHE_FLAG_WRITE)) {
                    return MemoryCheckNoWrite;
                }
                break;
            case MemoryOpDataRead:
                if (cacheFlags & CACHE_FLAG_NOREAD) {
                    return MemoryCheckNoRead;
                }
                break;
            default:
                break;
        }
        return cacheResult;
    }

    // Cache Miss -- check TLB
    uint32_t outAddress = 0;
    auto translateResult = translateAddress(address, count, asid, outAddress);
    if (translateResult.isComplete()) {
        // TLB Hit -- check for modes
        auto tlbFlags = tlb.getFlags(address, asid);
        switch (type) {
            case MemoryOpInstructionRead:
                if (tlbFlags & CACHE_FLAG_NOEXEC) {
                    return MemoryCheckNoExecute;
                }
                else if (tlbFlags & CACHE_FLAG_NOREAD) {
                    return MemoryCheckNoRead;
                }
                break;
            case MemoryOpDataWrite:
                if (!(tlbFlags & CACHE_FLAG_WRITE)) {
                    return MemoryCheckNoWrite;
                }
                break;
            case MemoryOpDataRead:
                if (tlbFlags & CACHE_FLAG_NOREAD) {
                    return MemoryCheckNoRead;
                }
                break;
            default:
                break;
        }
        return cacheResult;
    }
    else if (translateResult.result == MemoryCheckSegmentError) {
        // We're trying to cross a segment in one operation...
        return MemoryCheckSegmentError;
    }

    // TLB Miss
    return MemoryCheckNoPresent;
}

uint32_t MemoryUnit::read(CacheType type, uint32_t address, int count, uint32_t asid) {
    if (lastUncachedRead.isValid() && lastUncachedRead.inAddress == (lastUncachedRead.inAddress & address)) {
        uint32_t value = 0;
        for (auto iter = lastUncachedRead.data.crbegin(); iter != lastUncachedRead.data.crend(); iter++) {
            value <<= 8;
            value |= *iter;
        }

        lastUncachedRead.invalidate();
        return value;
    }

    if (caches[type].contains(address, count, asid) != CacheContainsNone) {
        return convert(caches[type].read(address, count, asid));
    }
    return 0;
}

uint32_t MemoryUnit::convert(std::vector<uint8_t> content) {
    uint32_t value = 0;

    for (auto iter = content.crbegin(); iter != content.crend(); iter++) {
        value <<= 8;
        value |= *iter;
    }

    return value;
}

uint16_t MemoryUnit::queueRead(MemoryReadType type, uint32_t address, int count, uint32_t asid) {
    auto alreadyQueued = isReadQueued(type, address, asid);
    if (alreadyQueued) {
        return alreadyQueued;
    }

    int bytes = count;
    CacheType cacheType = type == InstructionRead ? InstructionCache : DataCache;
    if (canCache(address, asid) && caches.contains(cacheType)) {
        address = address & ~caches[cacheType].getLineMask();
        bytes   =            caches[cacheType].getLineBytes();
    }

    MemoryOperation operation;
    operation.inAddress = address;
    // need to translate
    operation.outAddress = address;
    operation.asid    = asid;
    operation.bytes   = bytes;
    operation.type    = type == InstructionRead ? MemoryOpInstructionRead : MemoryOpDataRead;
    operation.committed = true;

    return queueOperation(operation);
}

uint16_t MemoryUnit::queueOperation(MemoryOperation operation) {
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
void MemoryUnit::commitOperation(uint16_t operationId) {
    for (MemoryOperation& op: queuedOperations) {
        if (op.operationId == operationId && op.isValid()) {
            op.committed = true;
            if (op.type == MemoryOpDataWrite) {
                applyWrite(op);
            }
            break;
        }
    }
}
void MemoryUnit::invalidateOperation(uint16_t operationId) {
    for (auto iter = queuedOperations.begin(); iter != queuedOperations.end(); iter++) {
        if (iter->operationId == operationId) {
            queuedOperations.erase(iter);
            break;
        }
    }
}

bool MemoryUnit::isOperationPrepared() {
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

BusOperation MemoryUnit::getBusOperation() {
    for (int i = queuedOperations.size() - 1; i >= 0; --i) {
        if (queuedOperations[i].isReady()) {
            auto op = queuedOperations[i];

            if (op.type == MemoryOpDataWrite) {
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

void MemoryUnit::applyWrite(MemoryOperation op) {
    for (auto c: {InstructionCache, DataCache, UnifiedL2Cache}) {
        if (caches.contains(c) && caches[c].contains(op.inAddress, op.data.size(), op.asid) != CacheContainsNone) {
            caches[c].write(op.inAddress, op.asid, op.data);
        }
    }
}

void MemoryUnit::ingestWord(uint16_t word) {
    if (!pendingOperation.isValid()) {
        // should we error? not sure.
        return;
    }

    uint8_t low = word & 0xff;
    uint8_t high = word >> 8;

    MemoryOperation& pending = pendingOperation;

    pending.data.push_back(low);
    pending.data.push_back(high);

    if (pending.data.size() == pending.bytes) {
        if (canCache(pending.inAddress, pending.asid)) {
            CacheType type = pending.type == MemoryOpInstructionRead ? InstructionCache : DataCache;
            caches[type].load(pending.inAddress, pending.asid, 0, pending.data);
        }
        else {
            lastUncachedRead = pending;
        }

        pending.invalidate();
    }
}

std::string MemoryUnit::describeQueuedOperations() {
    std::stringstream response;

    response << "C  VADDR     PADDR     T B   BYTES";

    for (auto iter = queuedOperations.cbegin(); iter != queuedOperations.cend(); iter++) {
        //if (!(iter->isValid())) {
            //continue;
        //}
        response << std::endl << (iter->committed ? 1 : 0) << "  ";
        response << std::setw(8) << std::setfill('0') << std::hex << iter->inAddress << "  " << iter->outAddress << "  ";
        response << iter->type << " " << std::setw(2) << std::dec << iter->bytes << " ";

        for (auto it = iter->data.cbegin(); it != iter->data.cend(); it++) {
            response << " " << std::setw(2) << std::setfill('0') << std::hex << (int)(*it);
        }
    }

    if (pendingOperation.isValid()) {
        response << std::endl << "p  ";
        response << std::setw(8) << std::setfill('0') << std::hex << pendingOperation.inAddress << "  " << pendingOperation.outAddress << "  ";
        response << pendingOperation.type << " " << std::setw(2) << std::dec << pendingOperation.bytes << " ";

        for (auto it = pendingOperation.data.cbegin(); it != pendingOperation.data.cend(); it++) {
            response << " " << std::setw(2) << std::setfill('0') << std::hex << (int) (*it);
        }
    }

    return response.str();
}

std::string MemoryUnit::listContents(std::stringstream &input) {
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
            if (caches.contains(DataCache) && caches[DataCache].contains(address, 2, asid)) {
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

bool MemoryUnit::canCache(uint32_t address, uint32_t asid) {
    auto segment = getSegment(address);
    if (segment == MemorySegmentK1) {
        // unmapped, uncached
        return false;
    }
    else if (segment == MemorySegmentK0) {
        // unmapped, cachable
        address &= 0x1fffffff;
        for (auto region: noCacheRegions) {
            if (region.first <= address && (region.first + region.second) >= address) {
                return false;
            }
        }
        return true;
    }

    // MemorySegmentK2, MemorySegmentU0
    // mapped, cachable
    if (tlb.contains(address, 1, asid) != CacheContainsNone) {
        uint16_t flags = tlb.getFlags(address, asid);
    }
    else {
    }

    return true;
}

MemoryCheck MemoryUnit::translateAddress(uint32_t address, int count, uint32_t asid, uint32_t &outAddress, bool updateLru) {
    auto memorySegment = checkSegment(address, count);
    if (memorySegment == MemorySegmentError) {
        return MemoryCheckSegmentError;
    }

    if (memorySegment == MemorySegmentK0 || memorySegment == MemorySegmentK1) {
        // 29-bit unmapped addresses
        outAddress = address & 0x1fffffff;
        return MemoryCheckContainsSingle;
    }

    uint32_t startPage = (address            ) & 0xfffff000;
    uint32_t endPage   = (address + count - 1) & 0xfffff000;

    // Check if we're spanning a page boundary
    if (startPage != endPage) {
        // ignoring for now. may not be necessary
    }

    auto tlbResult = tlb.contains(address, 1, asid);
    if (tlbResult == CacheContainsNone) {
        return MemoryCheckNoPresent;
    }

    auto tlbFlags = tlb.getFlags(address, asid);
    if (!(tlbFlags & CACHE_FLAG_PRES)) {
        return MemoryCheckNoPresent;
    }

    auto tlbContent = tlb.get(address, 1, asid, updateLru);
    uint32_t upperAddress = tlbContent[0];

    outAddress = (upperAddress << 12) | (address & 0xfff);

    return MemoryCheckContainsSingle;
}

MemorySegment MemoryUnit::checkSegment(uint32_t address, int count) {
    auto startSegment = getSegment(address);
    auto endSegment = getSegment(address + count - 1);
    if (startSegment != endSegment) {
        return MemorySegmentError;
    }
    return startSegment;
}

MemorySegment MemoryUnit::getSegment(uint32_t address) {
    if (     address >= 0xc0000000) {
        // Segment K2 - cached, mapped
        return MemorySegmentK2;
    }
    else if (address >= 0xa0000000) {
        // Segment K1 - uncached, unmapped
        return MemorySegmentK1;
    }
    else if (address >= 0x80000000) {
        // Segment K0 - cached, unmapped
        return MemorySegmentK0;
    }
    // Segment U0 - cached, mapped
    return MemorySegmentU0;
}

bool MemoryUnit::isKernelSegment(uint32_t address) {
    return getSegment(address) != MemorySegmentU0;
}

uint16_t MemoryUnit::isReadQueued(MemoryReadType type, uint32_t address, uint32_t asid) {
    MemoryOpType opType = type == InstructionRead ? MemoryOpInstructionRead : MemoryOpDataRead;
    for (auto op: queuedOperations) {
        if (op.type != opType || op.asid != asid) {
            continue;
        }
        if (op.inAddress <= address && (op.inAddress + op.bytes) >= address) {
            return op.operationId;
        }
    }

    if (pendingOperation.type == opType &&
        pendingOperation.asid == asid &&
        pendingOperation.inAddress <= address &&
        (pendingOperation.inAddress + pendingOperation.bytes) > address) {
        return pendingOperation.operationId;
    }

    return 0;
}

BusOperation MemoryOperation::getBusOperation() {
    BusOperation op;
    op.address = outAddress;
    op.isValid = true;
    op.isRead = type != MemoryOpDataWrite;
    op.bytes = bytes;

    bool hasData = data.size() > 0;

    uint16_t word;

    if ((outAddress & 1) || (bytes & 1)) {
        // we're misaligned somehow. split into multiple
        // bus operations
        if (outAddress & 1) {
            // we are byte aligned. chip off one
            // byte and send it
            op.bytes = 1;
            outAddress = (outAddress & 0xfffffffe) + 2;

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
                outAddress += 2;
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

CacheCheck MemoryOperation::contains(CacheType _type, uint32_t _address, int _count, uint32_t _asid) {
    if (isValid() && _asid == asid) {
        if (_type == InstructionCache && type == MemoryOpInstructionRead) {
        }
        else if (_type == DataCache && type == MemoryOpDataRead) {
            
        }
    }
    return CacheContainsNone;
}

MemoryCheck::MemoryCheck(CacheCheck check) {
    switch (check) {
        case CacheContainsPartial:
            result = MemoryCheckContainsPartial;
            break;
        case CacheContainsSingle:
            result = MemoryCheckContainsSingle;
            break;
        case CacheContainsSplit:
            result = MemoryCheckContainsSplit;
            break;
        case CacheContainsNone:
        default:
            result = MemoryCheckContainsNone;
            break;
    }
}

}; // namespace n16r

}; // namespace nbus

}; // namespace sysnp

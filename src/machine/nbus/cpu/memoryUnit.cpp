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

CacheCheck MemoryUnit::contains(CacheType type, uint32_t address, int count, uint32_t asid) {
    if (!canCache(address)) {
        if (lastUncachedRead.isValid() && lastUncachedRead.address == (lastUncachedRead.address & address)) {
            return CacheContainsSingle;
        }
        return CacheContainsNone;
    }

    auto pendingContains = pendingOperation.contains(type, address, count, asid);
    if (pendingContains != CacheContainsNone) {
        return pendingContains;
    }

    if (!caches.contains(type)) {
        return CacheContainsNone;
    }

    return caches[type].contains(address, count, asid);
}

uint32_t MemoryUnit::read(CacheType type, uint32_t address, int count, uint32_t asid) {
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
            if (op.type == MemoryOperationDataWrite) {
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
MemoryOperation MemoryUnit::getOperation() {
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

BusOperation MemoryUnit::getBusOperation() {
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

void MemoryUnit::applyWrite(MemoryOperation op) {
    for (auto c: {InstructionCache, DataCache, UnifiedL2Cache}) {
        if (contains(c, op.address, op.data.size(), op.asid) != CacheContainsNone) {
            caches[c].write(op.address, op.asid, op.data);
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

std::string MemoryUnit::describeQueuedOperations() {
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

bool MemoryUnit::canCache(uint32_t address) {
    for (auto region: noCacheRegions) {
        if (region.first <= address && (region.first + region.second) >= address) {
            return false;
        }
    }

    return true;
}

uint16_t MemoryUnit::isReadQueued(MemoryReadType type, uint32_t address, uint32_t asid) {
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
            address = (address & 0xfffffffe) + 2;

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

CacheCheck MemoryOperation::contains(CacheType _type, uint32_t _address, int _count, uint32_t _asid) {
    if (isValid() && _asid == asid) {
        if (_type == InstructionCache && type == MemoryOperationInstructionRead) {
        }
        else if (_type == DataCache && type == MemoryOperationDataRead) {
            
        }
    }
    return CacheContainsNone;
}

}; // namespace n16r

}; // namespace nbus

}; // namespace sysnp

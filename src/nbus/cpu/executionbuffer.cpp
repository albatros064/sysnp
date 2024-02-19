#include "n16r.h"
#include <iomanip>
#include <iostream>

namespace sysnp {

namespace nbus {

namespace n16r {

bool ExecutionBuffer::isEmpty() {
    return head == tail;
}

bool ExecutionBuffer::isFull() {
    return (tail + 3) % 4 == head;
}

uint8_t ExecutionBuffer::size() {
    return (head - tail) % 4;
}

bool ExecutionBuffer::hasAddress(uint32_t requestAddress) {
    if (isEmpty()) {
        return false;
    }

    for (uint8_t i = tail; (head - i) % 4 != 0; i = (i + 1) % 4) {
        if (requestAddress == address[i]) {
            return true;
        }
    }

    return false;
}

void ExecutionBuffer::addRequest(uint32_t request) {
    if (hasAddress(request) || (pendingRequest && requestedAddress == request)) {
        return;
    }

    requestedAddress = request;
    pendingRequest = true;
}

bool ExecutionBuffer::hasRequest() {
    return pendingRequest;
}

uint32_t ExecutionBuffer::getNextAddress() {
    if (pendingRequest) {
        return requestedAddress;
    }

    return address[(head + 3) % 4] + 2;
}

void ExecutionBuffer::checkBus(BusUnit &busUnit) {
    if (hasRequest() && busUnit.isReadReady() && !busUnit.isHighReadPriority()) {
        // check our request didn't change before it came back
        if (busUnit.getReadAddress() != requestedAddress) {
            // if it did, discard what we just got
            return;
        }
        push(requestedAddress, busUnit.getReadData());
        pendingRequest = false;
    }
}

uint16_t ExecutionBuffer::popTo(uint32_t request) {
    request = request & 0xfffffffe;

    uint16_t value;
    uint8_t i;
    for (i = tail; (head - i) % 4 != 0; i = (i + 1) % 4) {
        if (request == address[i]) {
            value = buffer[i];
            tail = i;
            break;
        }
    }

    return value;
}

void ExecutionBuffer::push(uint32_t setAddress, uint16_t word) {
    // not really sure where to swap the bytes, so we'll do that here
    uint8_t low = (uint8_t) ((word >> 8) & 0xff);
    buffer[head] = (word << 8) | low;
    address[head] = setAddress;
    uint8_t oldHead = head;
    head = (head + 1) % 4;

    if (pendingRequest && setAddress == requestedAddress) {
        if (head == tail) {
            tail = oldHead;
        }
        pendingRequest = false;
    }
}

void ExecutionBuffer::reset() {
    head = tail = 0;
    pendingRequest = false;
}
void ExecutionBuffer::output(std::stringstream &output) {
    for (int i = 0; i < 4; i++) {
        output << "B" << i << ": ";
        if (head == tail) {
            output << " ";
        }
        else if (i == head) {
            output << "h";
        }
        else if (i == tail) {
            output << "t";
        }
        else {
            output << " ";
        }

        output << " " << std::setw(8) << std::setfill('0') << std::hex << address[i];
        output << " " << std::setw(4) << std::setfill('0') << std::hex << buffer[i];
        output << std::endl;
    }
}

}; // namespace n16r

}; // namespace nbus

}; // namespace sysnp

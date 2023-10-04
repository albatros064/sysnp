#include "n16r.h"
#include <iostream>
#include <iomanip>

namespace sysnp {

namespace nbus {

namespace n16r {

N16R::N16R() {
}
N16R::~N16R() {
}

void N16R::init(const libconfig::Setting &setting) {
    setting.lookupValue("resetAddress", resetAddress);
    for (int i = 0; i < 8; i++) {
        registerFile[i] = altRegisterFile[i] = 0x5555;
    }
}

void N16R::postInit() {
    machine->debug("N16R::postInit()");
    busUnit.setBusInterface(interface);
    reset();
}

void N16R::clockUp() {
    machine->debug("N16R::clockUp()");
    // Execution core
    AluFunction aluFunction;
    switch (executionPhase) {
        case Fetch:
            machine->debug(" -Fetch");
            if (executionBuffer.hasAddress(instructionPointer)) {
                machine->debug(" --Decoding");
                uint16_t instructionWord = executionBuffer.popTo(instructionPointer);

                if (decoder.decode(instructionWord, instructionPointer)) {
                    machine->debug(" --Proceeding to fetch additional");
                    executionBuffer.addRequest(instructionPointer + 2);
                    executionPhase = ExecutionPhase::FetchAdditional;
                }
                else {
                    machine->debug(" --Proceeding to execute");
                    executionPhase = ExecutionPhase::Execute;
                }
            }
            else {
                machine->debug(" --Requesting");
                executionBuffer.addRequest(instructionPointer);
            }
            break;
        case FetchAdditional:
            machine->debug(" -FetchAdditional");
            if (executionBuffer.hasAddress(instructionPointer + 2)) {
                decoder.extraI = executionBuffer.popTo(instructionPointer + 2);
                executionPhase = ExecutionPhase::Execute;
            }
            break;
        case Execute:
            machine->debug(" -Execute");
            aluFunction = decoder.aluFunction;
            if (aluFunction != AluFunction::NoFunction) {
                machine->debug(" -- alu?");
                if (decoder.format == InstructionFormat::I) {
                    alu.load(registerFile[decoder.dReg], decoder.baseI);
                }
                else {
                    alu.load(registerFile[decoder.dReg], registerFile[decoder.sReg]);
                }
                registerFile[decoder.dReg] = alu.calculate(aluFunction, decoder.extend, false);
            }
            else if (decoder.format == InstructionFormat::R) {
                machine->debug(" -- R instruction");
                uint16_t tmp;
                switch (decoder.function) {
                    case 0:
                        registerFile[decoder.dReg] = registerFile[decoder.sReg];
                        break;
                    case 2:
                        tmp = registerFile[decoder.dReg];
                        registerFile[decoder.dReg] = registerFile[decoder.sReg];
                        registerFile[decoder.sReg] = tmp;
                        break;
                    case 3:
                        tmp = registerFile[decoder.dReg];
                        registerFile[decoder.dReg] = altRegisterFile[decoder.sReg];
                        altRegisterFile[decoder.sReg] = tmp;
                        break;
                    case 4:
                        registerFile[decoder.dReg] = altRegisterFile[decoder.sReg];
                        break;
                    case 5:
                        altRegisterFile[decoder.dReg] = registerFile[decoder.sReg];
                        break;
                    default:
                        // Invalid instruction
                        break;
                }
            }
            else if (decoder.format == InstructionFormat::M) {
                machine->debug(" --M instruction");
                addressCalculator.loadLow(decoder.sReg, decoder.baseI);
            }

            if (decoder.isDouble || decoder.format == InstructionFormat::M) {
                executionPhase = ExecutionPhase::ExecuteAdditional;
            }
            else {
                executionPhase = ExecutionPhase::Commit;
            }
            break;
        case ExecuteAdditional:
            machine->debug(" -ExecuteAdditional");
            if (decoder.format == InstructionFormat::M) {
                addressCalculator.loadHigh(registerFile[decoder.sReg+1]);
                uint32_t memoryAddress = addressCalculator.calculate();

                if (decoder.opcode == 2 || decoder.opcode == 3) {
                    machine->debug(" --adding read");
                    busUnit.addHighPriorityRead(memoryAddress);
                }
                else {
                    uint16_t writeData = registerFile[decoder.dReg];
                    uint8_t activeBytes = 3;
                    if (decoder.opcode == 4) {
                        if (memoryAddress & 1) {
                            activeBytes = 0x10;
                            writeData = writeData << 8;
                        }
                        else {
                            activeBytes = 0x01;
                            writeData = writeData & 0x00ff;
                        }
                    }
                    busUnit.addHighPriorityWrite(memoryAddress, writeData, activeBytes);
                }
            }
            else {
                aluFunction = decoder.aluFunction;
                if (aluFunction != AluFunction::NoFunction) {
                    ExtendFunction extend = ExtendFunction::NoExtend;
                    if (decoder.format == InstructionFormat::I) {
                        extend = ExtendFunction::SignAdditional;
                        alu.load(registerFile[decoder.dReg+1], decoder.baseI);
                    }
                    else {
                        alu.load(registerFile[decoder.dReg+1], registerFile[decoder.sReg+1]);
                    }
                    registerFile[decoder.dReg] = alu.calculate(aluFunction, extend, false);
                }
            }
            executionPhase = ExecutionPhase::Commit;
            break;
        case Commit:
            machine->debug(" -Commit");
            if (decoder.format == InstructionFormat::M) {
                if (decoder.opcode == 2 || decoder.opcode == 3) {
                    if (!busUnit.isReadReady() || !busUnit.isHighReadPriority()) {
                        machine->debug(" --Waiting for read");
                        break;
                    }
                    uint16_t readData = busUnit.getReadData();
                    if (decoder.opcode == 2) {
                        if (addressCalculator.calculate() & 1) {
                            readData = readData >> 8;
                        }
                        else {
                            readData = readData & 0x00ff;
                        }
                    }
                    registerFile[decoder.dReg] = readData;
                }
                instructionPointer = decoder.nextAddress;
                executionPhase = ExecutionPhase::Fetch;
            }
            else if (decoder.format == InstructionFormat::J) {
                instructionPointer =
                    (instructionPointer & 0xc000000) |
                    ((uint32_t) decoder.baseI << 17) |
                    ((uint32_t) decoder.extraI << 1);
                executionPhase = ExecutionPhase::Fetch;
            }
            else if (decoder.format == InstructionFormat::R && (decoder.function == 070 || decoder.function == 071)) {
                instructionPointer = registerFile[decoder.dReg & 0x6] << 16 | registerFile[decoder.dReg & 0x6 | 1];
                executionPhase = ExecutionPhase::Fetch;
            }
            else {
                if (decoder.format == InstructionFormat::B && core.branchTaken) {
                    uint32_t branchOffset = decoder.extraI;
                    if (branchOffset & 0x00008000) {
                        branchOffset |= 0xffff0000;
                    }
                    decoder.nextAddress += branchOffset;
                }

                if ((decoder.nextAddress & 0xffff0000) == (instructionPointer & 0xffff0000)) {
                    executionPhase = ExecutionPhase::Fetch;
                    instructionPointer = decoder.nextAddress;
                }
                else {
                    executionPhase = ExecutionPhase::CommitAdditional;
                }
            }
            break;
        case CommitAdditional:
            machine->debug(" -CommitAdditional");
            instructionPointer = decoder.nextAddress;
            executionPhase = ExecutionPhase::Fetch;
            break;
        default:
            break;
    }

    // Execution buffer
    machine->debug("Processing execution buffer");
    if ((!executionBuffer.isEmpty() && !executionBuffer.isFull()) || executionBuffer.hasRequest()) {
        busUnit.addLowPriorityRead(executionBuffer.getNextAddress());
    }

    // Bus interface
    machine->debug("Processing bus unit");
    busUnit.clockUp();
}

void N16R::clockDown() {
    machine->debug("N16R::clockDown()");

    machine->debug("Processing bus unit");
    busUnit.clockDown();

    machine->debug("Processing execution buffer");
    executionBuffer.checkBus(busUnit);
}

std::string N16R::command(std::stringstream &input) {
    std::stringstream response;

    std::string commandWord = "status";
    input >> commandWord;

    if (commandWord == "reset") {
        reset();
        response << "Ok.";
    }
    else if (commandWord == "status") {
        response << "MReg         AReg" << std::endl;
        for (int i = 0; i < 8; i++) {
            response << "0" << i << ": " << std::setw(6) << std::setfill('0') << std::oct << registerFile[i];
            response << "   ";
            response << "1" << i << ": " << std::setw(6) << std::setfill('0') << std::oct << altRegisterFile[i];
            response << std::endl;
        }
        response << "IP: " << std::setw(11) << std::setfill('0') << std::oct << instructionPointer << std::endl;
        executionBuffer.output(response);
        response << "Ok.";
    }
    else {
        response << "Ok.";
    }
    return response.str();
}

void N16R::reset() {
    instructionPointer = resetAddress;
    executionBuffer.reset();
    busUnit.reset();
    executionPhase = ExecutionPhase::Fetch;
}

bool DecoderUnit::decode(uint16_t word, uint32_t instructionPointer) {
    hasTrap = false;
    extend = ExtendFunction::NoExtend;
    opcode = (uint8_t) ((word & 0xf000) >> 12);
    nextAddress = instructionPointer + 2;
    if (opcode == 0 || opcode == 6) {
        if (opcode == 0) {
            format = InstructionFormat::R;
        }
        else {
            format = InstructionFormat::B;
            nextAddress += 2;
        }
        function = (word >> 0) & 077;
        dReg     = (word >> 9) & 007;
        sReg     = (word >> 6) & 007;
        baseI    = 0;
        extraI   = 0;
    }
    else if ((opcode & 0x7) == 0x7) {
        format   = InstructionFormat::J;
        function = 0;
        dReg     = 0;
        sReg     = 0;
        baseI    = 0;
        extraI   = 0;
        nextAddress += 2;
    }
    else if ((opcode & 0x8) || opcode == 1) {
        format   = InstructionFormat::I;
        function = (word >> 8) & 0x01;
        dReg     = (word >> 9) & 0x07;
        sReg     = 0;
        baseI    = (word >> 0) & 0xff;
        extraI   = 0;
    }
    else {
        format   = InstructionFormat::M;
        function = 0;
        dReg     = (word >> 9) & 0x07;
        sReg     = (word >> 6) & 0x06;
        baseI    = (word >> 0) & 0x7f;
        extraI   = 0;
    }

    isDouble = false;

    if (format == InstructionFormat::I) {
        switch (opcode) {
            case 001:
                isDouble = true;
                aluFunction = function ? AluFunction::Add : AluFunction::Sub;
                extend = ExtendFunction::Sign;
                break;
            case 010:
                aluFunction = AluFunction::Add;
                extend = function ? ExtendFunction::Zero : ExtendFunction::Sign;
                break;
            case 011:
                aluFunction = AluFunction::Sub;
                extend = function ? ExtendFunction::Zero : ExtendFunction::Sign;
                break;
            case 012:
                aluFunction = function ? AluFunction::Or: AluFunction::And;
                extend = ExtendFunction::Zero;
                break;
            case 013:
                aluFunction = function ? AluFunction::Lui : AluFunction::Xor;
                extend = ExtendFunction::Zero;
                break;
            case 014:
                aluFunction = AluFunction::LShift;
                break;
            case 015:
                aluFunction = function ? AluFunction::RShift : AluFunction::RShiftA;
                break;
            case 016:
                aluFunction = AluFunction::Sub;
                extend = function ? ExtendFunction::Zero : ExtendFunction::Sign;
                break;
            default:
                aluFunction = AluFunction::NoFunction;
                break;
        }
    }
    else if (format == InstructionFormat::R) {
        switch (function) {
            case 010:
                hasTrap = true;
            case 011:
                aluFunction = AluFunction::Add;
                break;
            case 012:
                hasTrap = true;
            case 013:
                aluFunction = AluFunction::Sub;
                break;
            case 014:
                aluFunction = AluFunction::Add;
                isDouble = true;
                break;
            case 020:
                aluFunction = AluFunction::And;
                break;
            case 021:
                aluFunction = AluFunction::Or;
                break;
            case 022:
                aluFunction = AluFunction::Xor;
                break;
            case 023:
                aluFunction = AluFunction::Nor;
                break;
            default:
                aluFunction = AluFunction::NoFunction;
                break;
        }
    }
    else {
        aluFunction = AluFunction::NoFunction;
    }

    return format == InstructionFormat::J || format == InstructionFormat::B;
}

void ArithmeticLogicUnit::load(uint16_t a, uint16_t b) {
    this->a = a;
    this->b = b;
}
uint16_t ArithmeticLogicUnit::calculate(AluFunction function, ExtendFunction extend, bool chain) {
    if (extend == ExtendFunction::SignAdditional) {
        if (b & 0x0080) {
            b = 0xffff;
        }
        else {
            b = 0x0000;
        }
    }
    else if (extend != ExtendFunction::NoExtend) {
        b = b & 0x00ff;
        if (extend == ExtendFunction::Sign && b & 0x0080) {
            b = b | 0xff00;
        }
    }

    uint32_t result;
    switch (function) {
        case Sub: // sub
            b = ~b + 1;
        case Add: // add
            result = (uint32_t) a + (uint32_t) b;
            if (chain) {
                result = result + overflow;
            }
            break;
        case And: // and
            result = a & b;
            break;
        case Or: // or
            result = a | b;
            break;
        case Xor: // xor
            result = a ^ b;
            break;
        case Nor: // nor
            result = ~(a | b);
            break;
        case LShift: // left shift
            b = b & 0xf + 1;
            result = a;
            result = (result << b) & 0xffff;
            break;
        case RShiftA: // right shift, arithmetic
            b = b & 0xf + 1;
            result = a;
            if (a & 0x8000) {
                result |= 0xffff0000;
            }
            result = (result >> b) & 0xffff;
            break;
        case RShift: // right shift, logical
            b = b & 0xf + 1;
            result = a;
            result = result >> b;
            break;
        case Lui: // lui
            result = (b & 0xff) << 8;
            break;
        default:
            result = 0;
            break;
    }

    overflow = (result & 0xffff0000) >  0;
    negative = (result & 0x00008000) >  0;
    zero     = (result & 0x0000ffff) == 0;

    return (uint16_t) (result & 0xffff);
}

bool ArithmeticLogicUnit::getOverflow() {
    return overflow;
}
bool ArithmeticLogicUnit::getNegative() {
    return negative;
}
bool ArithmeticLogicUnit::getZero() {
    return zero;
}

void AddressCalculator::loadLow(uint16_t low, uint16_t newOffset) {
    base = low;
    offset = newOffset & 0x7f;
    if (newOffset & 0x40) {
       offset = offset | 0xffffff80; 
    }
}
void AddressCalculator::loadHigh(uint16_t high) {
    base = base | ((uint32_t) high) << 16;
}
uint32_t AddressCalculator::calculate() {
    return base + offset;
}

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
    buffer[head] = word;
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

        output << " 0" << std::setw(11) << std::setfill('0') << std::oct << address[i];
        output << " 0" << std::setw(6) << std::setfill('0') << std::oct << buffer[i];
        output << std::endl;
    }
}

void BusUnit::setBusInterface(std::shared_ptr<NBusInterface> interface) {
    this->interface = interface;
}
void BusUnit::reset() {
    phase = BusPhase::Idle;
    readReady = false;
    readPriority = false;
    hasLowPriority = false;
    hasHighPriority = false;
    hasHighPriorityWrite = false;
}

void BusUnit::clockUp() {
    interface->deassert(NBusSignal::Address);
    interface->deassert(NBusSignal::Data);
    interface->deassert(NBusSignal::WriteEnable);
    interface->deassert(NBusSignal::ReadEnable);

    if (phase == BusPhase::Idle) {
        if (hasHighPriority) {
            interface->assert(NBusSignal::Address, highPriorityAddress);
            if (hasHighPriorityWrite) {
                interface->assert(NBusSignal::Data, highPriorityData);
                interface->assert(NBusSignal::WriteEnable, writeMode);
                phase = BusPhase::Write;
            }
            else {
                interface->assert(NBusSignal::ReadEnable, 1);
                phase = BusPhase::HighRead;
            }
        }
        else if (hasLowPriority) {
            interface->assert(NBusSignal::Address, lowPriorityAddress);
            interface->assert(NBusSignal::ReadEnable, 1);
            phase = BusPhase::LowRead;
        }
    }
}
void BusUnit::clockDown() {
    switch (phase) {
        case BusPhase::LowRead:
        case BusPhase::HighRead:
            if (phase == BusPhase::LowRead) {
                phase = BusPhase::LowReadWait;
                hasLowPriority = false;
            }
            else {
                phase = BusPhase::HighReadWait;
                hasHighPriority = false;
            }
            break;
        case BusPhase::LowReadWait:
        case BusPhase::HighReadWait:
            if (interface->sense(NBusSignal::NotReady)) {
                break;
            }

            readData = (uint16_t) (interface->sense(NBusSignal::Data) & 0xffff);
            readReady = true;
            readPriority = phase == BusPhase::HighReadWait;
            phase = BusPhase::Idle;
            break;
        case BusPhase::Write:
            phase = BusPhase::WriteWait;
            hasHighPriority = false;
            hasHighPriorityWrite = false;
            break;
        case BusPhase::WriteWait:
            if (interface->sense(NBusSignal::NotReady)) {
                break;
            }

            phase = BusPhase::Idle;
            break;
        case BusPhase::Idle:
        default:
            break;
    }
}

void BusUnit::addLowPriorityRead(uint32_t address) {
    lowPriorityAddress = address;
    hasLowPriority = true;
}
void BusUnit::addHighPriorityRead(uint32_t address) {
    highPriorityAddress = address;
    hasHighPriority = true;
    hasHighPriorityWrite = false;
}
void BusUnit::addHighPriorityWrite(uint32_t address, uint16_t data, uint8_t activeBytes) {
    highPriorityAddress = address;
    highPriorityData = data;
    hasHighPriority = true;
    hasHighPriorityWrite = true;
    writeMode = activeBytes;
}

bool BusUnit::isReadReady() {
    return readReady;
}
bool BusUnit::isHighReadPriority() {
    return readPriority;
}
uint16_t BusUnit::getReadData() {
    readReady = false;
    return readData;
}

}; // namespace n16r

}; // namespace nbus

}; // namespace sysnp


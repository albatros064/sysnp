#include "n16r.h"

namespace sysnp {

namespace nbus {

namespace n16r {

N16R::N16R() {
}
N16R::~N16R() {
}

void N16R::init(const libconfig::Setting &setting) {
    setting.lookupValue("resetAddress", resetAddress);
}

void N16R::postInit() {
    busUnit.setBusInterface(interface);
    reset();
}

void N16R::clockUp() {
    // Execution core
    switch (executionPhase) {
        case Fetch:
            if (executionBuffer.hasAddress(instructionPointer)) {
                uint16_t instructionWord = executionBuffer.popTo(instructionPointer);

                if (decoder.decode(instructionWord, instructionPointer)) {
                    executionBuffer.addRequest(instructionPointer + 2);
                    executionPhase = ExecutionPhase::FetchAdditional;
                }
                else {
                    executionPhase = ExecutionPhase::Execute;
                }
            }
            else {
                executionBuffer.addRequest(instructionPointer);
            }
            break;
        case FetchAdditional:
            if (executionBuffer.hasAddress(instructionPoiner + 2)) {
                decoder.extraI = executionBuffer.popTo(instructionPointer + 2);
                executionPhase = ExecutionPhase::Execute;
            }
            break;
        case Execute:
            AluFunction aluFunction = decoder.getAluFunction();
            if (aluFunction != AluFunction::None) {
                if (decoder.format == InstructionFormat::I) {
                    alu.load(registerFile[decoder.dReg], baseI);
                }
                else {
                    alu.load(registerFile[decoder.dReg], registerFile[decoder.sReg]);
                }
                registerFile[decoder.dReg] = alu.calculate(aluFunction);
            }
            else if (decoder.format == InstructionFormat::R) {
                switch (decoder.function) {
                    case 0:
                        registerFile[decoder.dReg] = registerFile[decoder.sReg];
                        break;
                    case 2:
                        uint16_t tmp = registerFile[decoder.dReg];
                        registerFile[decoder.dReg] = registerFile[decoder.sReg];
                        registerFile[decoder.sReg] = tmp;
                        break;
                    case 3:
                        uint16_t tmp = registerFile[decoder.dReg];
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

            if (decoder.executeAdditional) {
                executionPhase = ExecutionPhase::ExecuteAdditional;
            }
            else {
                executionPhase = ExecutionPhase::Commit;
            }
            break;
        case ExecuteAdditional:
            AluFunction aluFunction = decoder.getAluFunction();
            if (aluFunction != AluFunction::None) {
                alu.load(registerFile[decoder.dReg+1], 
            }
            executionPhase = ExecutionPhase::Commit;
            break;
        case Commit:
            if (format == InstructionFormat::J) {
                instructionPointer = instructionPointer & 0xc000000 | (baseI << 17) | (extraI << 1);
                executionPhase = ExecutionPhase::Fetch;
            }
            else if (format == InstructionFormat::R && (function == 070 || function == 071)) {
                instructionPointer = registerFile[dReg & 0x6] << 16 | registerFile[dreg & 0x6 | 1];
                executionPhase = ExecutionPhase::Fetch;
            }
            else {
                if (format == InstructionFormat::B && core.branchTaken) {
                    uint32_t branchOffset = decoder.extraI;
                    if (branchOffset & 0x00008000) {
                        branchOffset |= 0xffff0000;
                    }
                    decoder.nextAddress += branchOffset;
                }

                if (decoder.nextAddress & 0xffff0000 == instructionPointer & 0xffff0000) {
                    executionPhase = ExecutionPhase::Fetch;
                }
                else {
                    executionPhase = ExecutionPhase::CommitAdditional;
                }
            }
            break;
        case CommitAdditional:
            executionPhase = ExecutionPhase::Fetch;
            break;
        default:
            break;
    }

    // Execution buffer
    if ((!executionBuffer.isEmpty() && !executionBuffer.isFull()) || executionBuffer.hasRequest()) {
        busUnit.addLowPriorityRequest(executionBuffer.getNextAddress());
    }

    // Bus interface
    
}

void N16R::clockDown() {
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
        response << "Ok.";
    }
    else {
        response << "Ok.";
    }
    return response
}

void N16R::reset() {
    instructionPoiner = resetAddress;
    executionBuffer.reset()
    executionPhase = ExecutionPhase::InstructionFetch;
}

bool Decoder::decode(uint16_t word, uint32_t instructionPointer) {
    opcode = (word & 0xf000) >> 12;
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
    else if (opcode & 0x7 == 0x7) {
        format   = InstructionFormat::J;
        function = 0;
        dReg     = 0;
        sReg     = 0;
        baseI    = 0;
        extraI   = 0;
        nextAddress += 2;
    }
    else if (opcode & 0x8 || opcode == 1) {
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
                break;
            case 010:
                aluFunction = AluFunction::Add;
                break;
            case 011:
                aluFunction = AluFunction::Sub;
                break;
            case 012:
                aluFunction = function ? AluFunction::And : AluFunction::Or;
                break;
            case 013:
                aluFunction = function ? AluFunction::Xor : AluFunction::Lui;
                break;
            case 014:
                aluFunctino = AluFunction::LShift;
                break;
            case 015:
                aluFunction = function ? AluFunction::RShift : AluFunction::RShiftA;
                break;
            case 016:
                aluFunction = AluFunction::Sub;
                break;
            default:
                aluFunction = AluFunction::None;
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
                aluFunction = AluFunction::None;
                break;
        }
    }
    else {
        aluFunction = AluFunction::None;
    }

    return format == InstructionFormat::J || InstructionFormat::B;
}

void ALU::load(uint16_t a, uint16_t b) {
    this->a = a;
    this->b = b;
}
uint16_t ALU::calculate(AluFunction function, bool chain) {
    uint8_t extend = (function & 0xc0) >> 6;
    if (extend) {
        b = b & 0x00ff;
        if (extend == 2 && b & 0x0080) {
            b = b | 0xff00;
        }
    }

    function = function & 0x3f;

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
            result = (result >> b) 0xffff;
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

bool ALU::getOverflow() {
    return overflow;
}
bool ALU::getNegative() {
    return negative;
}
bool ALU::getZero() {
    return zero;
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

    for (uint8_t i = tail; (head - i) % 4 > 0; i = (i + 1) % 4) {
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

    return address[(head - 1) % 4] + 2;
}

uint16_t ExecutionBuffer::popTo(uint32_t request) {
    request = request & 0xfffffffe;

    uint16_t value;
    uint8_t i;
    for (i = tail; (head - i) % 4 > 0; i = (i + 1) % 4) {
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

}; // namespace n16r

}; // namespace nbus

}; // namespace sysnp


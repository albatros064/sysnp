#include "n16r.h"
#include <iostream>
#include <iomanip>

namespace sysnp {

namespace nbus {

namespace n16r {

N16R::N16R() {}
N16R::~N16R() {}

void N16R::init(const libconfig::Setting &setting) {
    setting.lookupValue("resetAddress", resetAddress);
    for (int i = 0; i < 8; i++) {
        registerFile[i] = altRegisterFile[i] = 0x5555;
    }
    sysRegisterFile[0] = 0;
    sysRegisterFile[1] = 0;
    pendingException = ExceptionType::ExceptionNone;
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

                auto result = decoder.decode(instructionWord, instructionPointer);
                if (result > 1) {
                    machine->debug(" --Proceeding to fetch additional");
                    executionBuffer.addRequest(instructionPointer + 2);
                    executionPhase = ExecutionPhase::FetchAdditional;
                }
                else if (result == 1) {
                    machine->debug(" -Reserved on decode " + std::to_string(decoder.function));
                    pendingException = ExceptionType::ReservedInstruction;
                    executionPhase = ExecutionPhase::Exception;
                }
                else {
                    machine->debug(" --Proceeding to execute");
                    executionPhase = ExecutionPhase::Execute;
                }
                exceptionSuppress = false;
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

                uint16_t result = alu.calculate(aluFunction, decoder.extend, false);
                if (decoder.format != InstructionFormat::B) {
                    registerFile[decoder.dReg] = result;
                }
            }
            else if (decoder.format == InstructionFormat::R) {
                machine->debug(" -- R instruction");
                uint16_t tmp;
                switch (decoder.function) {
                    case 000: // mov r16->r16
                    case 001: // mov r32->r32
                        registerFile[decoder.dReg] = registerFile[decoder.sReg];
                        break;
                    case 002: // xch r16<>r16
                    case 003: // xch r32<>r32
                        tmp = registerFile[decoder.dReg];
                        registerFile[decoder.dReg] = registerFile[decoder.sReg];
                        registerFile[decoder.sReg] = tmp;
                        break;
                    case 040: // mov a16->r16
                    case 042: // mov a32->r32
                        registerFile[decoder.dReg] = altRegisterFile[decoder.sReg];
                        break;
                    case 041: // mov r16->a16
                    case 043: // mov r32->a32
                        altRegisterFile[decoder.dReg] = registerFile[decoder.sReg];
                        break;
                    case 044: // xch r16<>a16
                    case 045: // xch r32<>a32
                        tmp = registerFile[decoder.dReg];
                        registerFile[decoder.dReg] = altRegisterFile[decoder.sReg];
                        altRegisterFile[decoder.sReg] = tmp;
                        break;
                    case 030: // mov s16->r16
                    case 032: // mov s32->r32
                        if (!isKernel()) {
                            pendingException = ExceptionType::ReservedInstruction;
                            break;
                        }
                        registerFile[decoder.dReg] = sysRegisterFile[decoder.sReg];
                        break;
                    case 031: // mov r16->s16
                    case 033: // mov r32->s32
                        if (!isKernel()) {
                            pendingException = ExceptionType::ReservedInstruction;
                            break;
                        }
                        sysRegisterFile[decoder.dReg] = registerFile[decoder.sReg];
                        break;
                    case 050: // syscall
                        pendingException = ExceptionType::Syscall;
                        break;
                    case 051: // eret
                        if (!isKernel()) {
                            pendingException = ExceptionType::ReservedInstruction;
                            break;
                        }
                        // shift the kernel state stack
                        tmp = sysRegisterFile[1] & 0xfff0 | ((sysRegisterFile[1] >> 2) & 0xf);
                        sysRegisterFile[1] = tmp;
                        exceptionSuppress = true;
                        break;
                    case 070: // jr r32
                    case 071: // jr a32
                    case 072: // jalr
                        // avoid throwing an invalid instruction exception
                        break;
                    default:
                        // Invalid instruction
                        pendingException = ExceptionType::InvalidInstruction;
                        machine->debug(" -Invalid Instruction " + std::to_string(decoder.function));
                        break;
                }
            }
            else if (decoder.format == InstructionFormat::M) {
                machine->debug(" --M instruction");
                addressCalculator.loadLow(registerFile[decoder.sReg << 1], decoder.extraI);
            }
            else if (decoder.format == InstructionFormat::E) {
                machine->debug(" --E instruction");

                addressCalculator.loadLow(registerFile[decoder.sReg << 1], registerFile[decoder.rReg]);
            }

            if (decoder.isDouble || decoder.format == InstructionFormat::B) {
                executionPhase = ExecutionPhase::ExecuteAdditional;
            }
            else if (decoder.format == InstructionFormat::M || decoder.format == InstructionFormat::E) {
                executionPhase = ExecutionPhase::ExecuteAdditional;
            }
            else {
                executionPhase = ExecutionPhase::Commit;
            }

            if (pendingException != ExceptionType::ExceptionNone) {
                machine->debug(" -Going to Exception");
                executionPhase = ExecutionPhase::Exception;
            }

            break;
        case ExecuteAdditional:
            machine->debug(" -ExecuteAdditional");
            if (decoder.format == InstructionFormat::M || decoder.format == InstructionFormat::E) {
                addressCalculator.loadHigh(registerFile[(decoder.sReg << 1) + 1]);
                uint32_t memoryAddress = addressCalculator.calculate();

                // b0x : read
                // b1x : write
                if (decoder.function & 2) {
                    uint16_t writeData = registerFile[decoder.dReg];
                    uint8_t activeBytes = 0b11;
                    // bx0 : byte
                    // bx1 : word
                    if (!(decoder.function & 1)) {
                        if (memoryAddress & 1) {
                            activeBytes = 0b10;
                            writeData = writeData << 8;
                        }
                        else {
                            activeBytes = 0b01;
                            writeData = writeData & 0x00ff;
                        }
                    }
                    busUnit.addHighPriorityWrite(memoryAddress, writeData, activeBytes);
                }
                else {
                    machine->debug(" --adding read");
                    busUnit.addHighPriorityRead(memoryAddress);
                }
            }
            else if (decoder.format == InstructionFormat::B) {
                // we emulate an extra cycle for decision time....
                machine->debug(" --branch decision cycle");
            }
            else {
                aluFunction = decoder.aluFunction;
                if (aluFunction != AluFunction::NoFunction) {
                    ExtendFunction extend = ExtendFunction::NoExtend;
                    if (decoder.format == InstructionFormat::I) {
                        extend = ExtendFunction::SignAdditional;
                        alu.load(registerFile[decoder.dReg + 1], decoder.baseI);
                    }
                    else {
                        alu.load(registerFile[decoder.dReg + 1], registerFile[decoder.sReg + 1]);
                    }

                    registerFile[decoder.dReg + 1] = alu.calculate(aluFunction, extend, false);
                }
                else if (decoder.format == InstructionFormat::R) {
                    uint16_t tmp;
                    switch (decoder.function) {
                        case 001: // mov r32->r32
                            registerFile[decoder.dReg + 1] = registerFile[decoder.sReg + 1];
                            break;
                        case 003: // xch r32<>r32
                            tmp = registerFile[decoder.dReg + 1];
                            registerFile[decoder.dReg + 1] = registerFile[decoder.sReg + 1];
                            registerFile[decoder.sReg + 1] = tmp;
                            break;
                        case 032: // mov s32->r32
                            registerFile[decoder.dReg + 1] = sysRegisterFile[decoder.sReg + 1];
                            break;
                        case 033: // mov r32->s32
                            sysRegisterFile[decoder.dReg + 1] = registerFile[decoder.sReg + 1];
                            break;
                        case 042: // mov a32->r32
                            registerFile[decoder.dReg + 1] = altRegisterFile[decoder.sReg + 1];
                            break;
                        case 043: // mov r32->a32
                            altRegisterFile[decoder.dReg + 1] = registerFile[decoder.sReg + 1];
                            break;
                        case 045: // xch r32<>r32
                            tmp = registerFile[decoder.dReg + 1];
                            registerFile[decoder.dReg + 1] = altRegisterFile[decoder.sReg + 1];
                            altRegisterFile[decoder.sReg + 1] = tmp;
                            break;
                        default:
                            break;
                    }
                }
            }
            executionPhase = ExecutionPhase::Commit;
            break;
        case Commit:
            machine->debug(" -Commit");
            if (decoder.format == InstructionFormat::M || decoder.format == InstructionFormat::E) {
                // b0x : read
                // b1x : write
                if (!(decoder.function & 2)) {
                    if (!busUnit.isReadReady() || !busUnit.isHighReadPriority()) {
                        machine->debug(" --Waiting for read");
                        break;
                    }
                    uint16_t readData = busUnit.getReadData();
                    // bx0 : byte
                    // bx1 : word
                    if (!(decoder.function & 1)) {
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
                // handle jal
                if (decoder.opcode == 7) {
                    registerFile[14] = ( decoder.nextAddress        & 0xffff);
                    registerFile[15] = ((decoder.nextAddress >> 16) & 0xffff);
                }

                instructionPointer =
                    (instructionPointer & 0xc0000000) |
                    (((uint32_t) decoder.baseI ) << 17) |
                    (((uint32_t) decoder.extraI) <<  1);
                executionPhase = ExecutionPhase::Fetch;
            }
            else if (decoder.format == InstructionFormat::R && (decoder.function == 070 || decoder.function == 071 || decoder.function == 072)) {
                // start with a simple jr d.32
                uint16_t rl = registerFile[ decoder.dReg << 1     ];
                uint16_t rh = registerFile[(decoder.dReg << 1) + 1];

                // oops, no it's a jr a.32
                if (decoder.function == 071) {
                    rl = altRegisterFile[ decoder.dReg << 1     ];
                    rh = altRegisterFile[(decoder.dReg << 1) + 1];
                }

                // just kidding, we're jalr d.32
                if (decoder.function == 072) {
                    // store the link pointer
                    registerFile[14] = ( decoder.nextAddress        & 0xffff);
                    registerFile[15] = ((decoder.nextAddress >> 16) & 0xffff);
                }

                instructionPointer = ((uint32_t) rh) << 16 | rl;
                executionPhase = ExecutionPhase::Fetch;
            }
            else {
                if (decoder.format == InstructionFormat::B && alu.branchTaken(decoder.branchFunction)) {
                    machine->debug(" --branch taken");
                    uint32_t branchOffset = decoder.extraI;
                    if (branchOffset & 0x00008000) {
                        branchOffset |= 0xffff0000;
                    }
                    branchOffset <<= 1;
                    decoder.nextAddress += branchOffset - 4;
                }

                if ((decoder.nextAddress & 0xffff0000) == (instructionPointer & 0xffff0000)) {
                    executionPhase = ExecutionPhase::Fetch;
                    instructionPointer = decoder.nextAddress;
                }
                else {
                    // We're simulating an extra cycle for the address carry.
                    executionPhase = ExecutionPhase::CommitAdditional;
                }
            }
            break;
        case CommitAdditional:
            machine->debug(" -CommitAdditional");
            // We're simulating an extra cycle for the address carry.
            instructionPointer = decoder.nextAddress;
            executionPhase = ExecutionPhase::Fetch;
            break;
        case Exception:
            machine->debug(" -Exception of type " + std::to_string(pendingException));
            // set exception code
            sysRegisterFile[0 ] = sysRegisterFile[0] & 0xff00 | (pendingException << 2);
            // set execution state to 0,0 (kernel mode, exceptions disabled)
            sysRegisterFile[1 ] = sysRegisterFile[1] & 0xffc0 | (sysRegisterFile[1] & 0xf) << 2;
            // epc
            sysRegisterFile[14] = (uint16_t) (instructionPointer & 0xffff);
            sysRegisterFile[15] = (uint16_t) (instructionPointer >> 16);

            pendingException = ExceptionType::ExceptionNone;

            // jump to the exception handler (register $65)
            instructionPointer = ((uint32_t) sysRegisterFile[11]) << 16 | sysRegisterFile[10];

            executionPhase = ExecutionPhase::Fetch;
            break;
        default:
            break;
    }

    // are interrupts enabled and not suppressed?
    if (sysRegisterFile[1] & 1 && !exceptionSuppress) {
        // are we at the end of an instruction?
        if (executionPhase == ExecutionPhase::Fetch || executionPhase == ExecutionPhase::Exception) {
            // do we have pending interrupts?
            if (sysRegisterFile[0] & 0xff00) {
                executionPhase = ExecutionPhase::Exception;
                pendingException = ExceptionType::Interrupt;
            }
        }
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

    uint16_t pendingInterrupts = busUnit.hasInterrupt();
    pendingInterrupts <<= 8;
    pendingInterrupts &= sysRegisterFile[1];
    sysRegisterFile[0] = (sysRegisterFile[0] & 0xff) | pendingInterrupts;

    machine->debug("Processing execution buffer");
    executionBuffer.checkBus(busUnit);

    //std::stringstream rude;
    //rude << "status";
    //std::cout << command(rude) << std::endl;
}

bool N16R::isKernel() {
    return (sysRegisterFile[1] & 0x2) == 0;
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
        response << "MReg           AReg           SReg" << std::endl;
        for (int i = 0; i < 8; i++) {
            response << "0" << i << ": " << std::setw(4) << std::setfill('0') << std::hex << registerFile[i];
            response << "       ";
            response << "1" << i << ": " << std::setw(4) << std::setfill('0') << std::hex << altRegisterFile[i];
            response << "       ";
            response << "2" << i << ": " << std::setw(4) << std::setfill('0') << std::hex << sysRegisterFile[i];
            response << std::endl;
        }
        for (int i = 0; i < 8; i++) {
            int e = i << 1;
            uint32_t r = registerFile[e + 1];
            r = (r << 16) | registerFile[e];
            response << "4" << i << ": " << std::setw(8) << std::setfill('0') << std::hex << r << "   ";
            r = altRegisterFile[e + 1];
            r = (r << 16) | altRegisterFile[e];
            response << "5" << i << ": " << std::setw(8) << std::setfill('0') << std::hex << r << "   ";
            r = sysRegisterFile[e + 1];
            r = (r << 16) | sysRegisterFile[e];
            response << "6" << i << ": " << std::setw(8) << std::setfill('0') << std::hex << r;
            response << std::endl;
        }
        response << "IP: " << std::setw(8) << std::setfill('0') << std::hex << instructionPointer << std::endl;
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
    sysRegisterFile[1] = 0;
    executionBuffer.reset();
    busUnit.reset();
    executionPhase = ExecutionPhase::Fetch;
    exceptionSuppress = false;
}

int DecoderUnit::decode(uint16_t word, uint32_t instructionPointer) {
    hasTrap = false;
    extend = ExtendFunction::NoExtend;
    opcode = (uint8_t) ((word & 0xf000) >> 12);
    nextAddress = instructionPointer + 2;
    if (opcode == 0 || opcode == 2 || opcode == 6) {
        if (opcode == 0) {
            format = InstructionFormat::R;
        }
        else {
            if (opcode == 2) {
                format = InstructionFormat::M;
            }
            else {
                format = InstructionFormat::B;
            }
            nextAddress += 2;
        }

        function = (word >> 0) & 077;
        dReg     = (word >> 9) & 007;
        sReg     = (word >> 6) & 007;
        rReg     = 0;
        baseI    = 0;
        extraI   = 0;
    }
    else if ((opcode & 0x7) == 0x7) {
        format   = InstructionFormat::J;
        function = 0;
        dReg     = 0;
        sReg     = 0;
        rReg     = 0;
        baseI    = (word >> 0) & 0x0fff;
        extraI   = 0;
        nextAddress += 2;
    }
    else if ((opcode & 0x8) || opcode == 1) {
        format   = InstructionFormat::I;
        function = (word >> 8) & 0x01;
        dReg     = (word >> 9) & 0x07;
        sReg     = 0;
        rReg     = 0;
        baseI    = (word >> 0) & 0xff;
        extraI   = 0;
    }
    else if (opcode == 3) {
        format   = InstructionFormat::E;
        function = (word >> 0) & 007;
        dReg     = (word >> 9) & 007;
        sReg     = (word >> 6) & 007;
        rReg     = (word >> 3) & 007;
        baseI    = 0;
        extraI   = 0;
    }
    else {
        return 1;
    }

    isDouble = false;

    if (format == InstructionFormat::B) {
        aluFunction = AluFunction::Sub;
        switch (function) {
            case 0:
                branchFunction = BranchFunction::Equal;
                break;
            case 1:
                branchFunction = BranchFunction::NotEqual;
                break;
            case 2:
                branchFunction = BranchFunction::GreaterThan;
                break;
            case 3:
                branchFunction = BranchFunction::NotGreater;
                break;
            case 4:
                branchFunction = BranchFunction::LessThan;
                break;
            case 5:
                branchFunction = BranchFunction::NotLess;
            default:
                // undefined instruction
                break;
        }
    }
    else if (format == InstructionFormat::I) {
        switch (opcode) {
            case 001:
                isDouble = true;
                aluFunction = function ? AluFunction::Sub : AluFunction::Add;
                extend = ExtendFunction::Sign;
                dReg <<= 1;
                sReg <<= 1;
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
                aluFunction = function ? AluFunction::Lli : AluFunction::Lui;
                extend = ExtendFunction::Zero;
                break;
            case 013:
                aluFunction = function ? AluFunction::Or : AluFunction::And;
                extend = ExtendFunction::Zero;
                break;
            case 014:
                aluFunction = function ? AluFunction::LShift : AluFunction::Xor;
                if (!function) {
                    extend = ExtendFunction::Zero;
                }
                break;
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
            case 001: // mov.32 D->D
            case 003: // xch.32 D<>D
            case 032: // mov.32 S->D
            case 033: // mov.32 D->S
            case 042: // mov.32 A->D
            case 043: // mov.32 D->A
            case 045: // xch.32 D<>A
                aluFunction = AluFunction::NoFunction;
                isDouble = true;
                break;
            case 010: // add.16
                hasTrap = true;
            case 011: // addu.16
                aluFunction = AluFunction::Add;
                break;
            case 012: // sub.16
                hasTrap = true;
            case 013: // subu.16
                aluFunction = AluFunction::Sub;
                break;
            case 014: // add.32
                hasTrap = true;
            case 015: // addu.32
                aluFunction = AluFunction::Add;
                isDouble = true;
                break;
            case 016: // sub.32
                hasTrap = true;
            case 017: // subu.32
                aluFunction = AluFunction::Sub;
                isDouble = true;
                break;
            case 020: // and.16
                aluFunction = AluFunction::And;
                break;
            case 021: // or.16
                aluFunction = AluFunction::Or;
                break;
            case 022: // xor.16
                aluFunction = AluFunction::Xor;
                break;
            case 023: // nor.16
                aluFunction = AluFunction::Nor;
                break;
            case 000: // mov.16 D->D
            case 002: // xch.16 D<>D
            case 030: // mov.16 S->D
            case 031: // mov.16 D->S
            case 040: // mov.16 A->D
            case 041: // mov.16 D->A
            case 044: // mov.16 D<>A
            case 050: // syscall
            case 051: // eret
            //se 052: // hlt
            case 070: // jr D
            case 071: // jr A
            case 072: // jalr D
            default:
                aluFunction = AluFunction::NoFunction;
                break;
        }
        if (isDouble) {
            dReg <<= 1;
            sReg <<= 1;
        }
    }
    else {
        aluFunction = AluFunction::NoFunction;
    }

    return (format == InstructionFormat::J || format == InstructionFormat::B || format == InstructionFormat::M) ? 2 : 0;
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
            b = (b & 0xf) + 1;
            result = a;
            result = (result << b);// & 0xffff;
            break;
        case RShiftA: // right shift, arithmetic
            b = (b & 0xf) + 1;
            result = a;
            if (a & 0x8000) {
                result |= 0xffff0000;
            }
            result = (result >> b) & 0xffff;
            break;
        case RShift: // right shift, logical
            b = (b & 0xf) + 1;
            result = a;
            result = result >> b;
            break;
        case Lui: // lui
            result = (b & 0xff) << 8;
            break;
        case Lli: // lli
            result = b & 0xff;
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
bool ArithmeticLogicUnit::branchTaken(BranchFunction func) {
    switch (func) {
        case Equal:
            return zero;
        case NotEqual:
            return !zero;
        case GreaterThan:
            return !zero && !negative;
        case NotGreater:
            return zero || negative;
        case LessThan:
            return negative;
        case NotLess:
            return !negative;
        default:
            return false;
    }
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
    offset = newOffset;
    if (offset & 0x8000) {
       offset |= 0xffff0000; 
    }
}
void AddressCalculator::loadHigh(uint16_t high) {
    base = base | ((uint32_t) high) << 16;
}
uint32_t AddressCalculator::calculate() {
    return base + offset;
}

}; // namespace n16r

}; // namespace nbus

}; // namespace sysnp


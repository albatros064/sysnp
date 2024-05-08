#include "n16r.h"
#include <iostream>
#include <iomanip>

namespace sysnp {

namespace nbus {

namespace n16r {

void N16R::init(const libconfig::Setting &setting) {
    isPipelined = false;
    setting.lookupValue("resetAddress", resetAddress);
    setting.lookupValue("pipelined", isPipelined);

    const libconfig::Setting& cNoCache = setting["noCache"];
    int regionCount = cNoCache.getLength();
    for (int i = 0; i < regionCount; i++) {
        const libconfig::Setting& cRegion = cNoCache[i];
        int start = -1;
        int size = -1;

        cRegion.lookupValue("start", start);
        cRegion.lookupValue("size", size);

        if (start >= 0) {
            cacheController.addNoCacheRegion(start, size);
        }
    }

    registerFile[040] = 0;
    registerFile[041] = 0;

    cacheController.setCache(InstructionCache, 32, 4, 5, 2);
    cacheController.setCache(DataCache, 32, 4, 5, 2);
}

void N16R::postInit() {
    machine->debug("N16R::postInit()");
    busUnit.setBusInterface(interface);
    reset();
}

void N16R::clockUp() {
    machine->debug("N16R::clockUp()");

    fetchStage();
    decodeStage();
    executeStage();
    memoryStage();
    writeBackStage();

    // Bus interface
    machine->debug("Processing bus unit");

    if (busUnit.isIdle()) {
        if (cacheController.isOperationPrepared()) {
            busUnit.queueOperation(cacheController.getOperation().asBusOperation());
        }
    }
    busUnit.clockUp();
}

void N16R::clockDown() {
    machine->debug("N16R::clockDown()");

    machine->debug("Shifting stages");
    stageShift();

    machine->debug("Processing bus unit");
    busUnit.clockDown();

    if (busUnit.hasData()) {
        cacheController.ingestWord(busUnit.getWord());
    }

    uint16_t pendingInterrupts = busUnit.hasInterrupt();
    pendingInterrupts <<= 8;
    pendingInterrupts &= registerFile[041];
    registerFile[040] = (registerFile[040] & 0xff) | pendingInterrupts;

    machine->debug("Processing execution buffer");
}

void N16R::fetchStage() {
    if (stageRegisters.size() < 1 || stageRegisters[0].bubble) {
        return;
    }

    StageRegister stage = stageRegisters[0];

    uint32_t nextWord = stage.instructionPointer + 2;

    if (stage.delayed) {
        if (stage.instructionPointer != stage.nextInstructionPointer) {
            if (cacheController.contains(InstructionCache, nextWord, 0)) {
                stage.regVals[1] = cacheController.read(InstructionCache, nextWord, 0);
                stage.delayed = false;
            }
            else {
                cacheController.queueRead(InstructionRead, nextWord, 0);
                stage.delayed = true;
            }
        }
    }
    else {
        if (cacheController.contains(InstructionCache, stage.instructionPointer, 0)) {
            stage.regVals[0] = cacheController.read(InstructionCache, stage.instructionPointer, 0);
            stage.delayed = false;
        }
        else {
            cacheController.queueRead(InstructionRead, stage.instructionPointer, 0);
            stage.delayed = true;
        }
    }

    if (!stage.delayed) {
        // early-decode
        uint8_t opcode = (stage.regVals[0] >> 12) & 0xf;
        if (opcode == 002 || opcode == 006 || opcode == 007 || opcode == 017) {
            stage.nextInstructionPointer += 4;
            stage.altInstructionPointer = stage.nextInstructionPointer;

            if (cacheController.contains(InstructionCache, nextWord, 0)) {
                stage.regVals[1] = cacheController.read(InstructionCache, nextWord, 0);
            }
            else {
                cacheController.queueRead(InstructionRead, nextWord, 0);
                stage.delayed = true;
            }
        }
        else {
            stage.nextInstructionPointer += 2;
            stage.altInstructionPointer = stage.nextInstructionPointer;
        }

        if (!stage.delayed && (opcode == 007 || opcode == 017)) {
            uint32_t baseI = stage.regVals[0] & 0xfff;
            uint32_t extrI = stage.regVals[1];
            stage.nextInstructionPointer = stage.instructionPointer & 0xe0000000 | (baseI << 17) | (extrI << 1);
        }
    }

    stageRegisters[0] = stage;
}

void N16R::decodeStage() {
    if (stageRegisters.size() < 2 || stageRegisters[1].bubble) {
        return;
    }

    auto stage = stageRegisters[1];

    stage.commitOp = CommitNop;

    auto instruction = stage.regVals[0];
    uint8_t opcode = (instruction >> 12) & 0xf;

    bool isPrivileged = false;

    if (opcode == 000) {
        // R

        uint8_t aReg = (instruction >> 9) & 07;
        uint8_t bReg = (instruction >> 6) & 07;
        uint8_t func = instruction & 077;

        stage.executeCanOverflow = false;
        stage.commitOp = CommitWriteBack;

        bool isDouble = false;
        bool isCustom = false;

        uint8_t aRegBank = 0;
        uint8_t bRegBank = 0;

        bool isExchange = false;

        uint32_t target;

        switch (func) {
            case 001: // mov.32 D->D
                isDouble = true;
            case 000: // mov.16 D->D
                break;
            case 003: // xch.32 D<>D
                isDouble = true;
            case 002: // xch.16 D<>D
                isExchange = true;
                break;

            case 010: // add.16
                stage.executeCanOverflow = true;
            case 011: // addu.16
                stage.executeOp = ExecuteAdd;
                break;
            case 012: // sub.16
                stage.executeCanOverflow = true;
            case 013: // subu.16
                stage.executeOp = ExecuteSubtract;
                break;
            case 014: // add.32
                stage.executeCanOverflow = true;
            case 015: // addu.32
                stage.executeOp = ExecuteAdd;
                isDouble = true;
                break;
            case 016: // sub.32
                stage.executeCanOverflow = true;
            case 017: // subu.32
                stage.executeOp = ExecuteSubtract;
                isDouble = true;
                break;

            case 020: // and.16
                stage.executeOp = ExecuteAnd;
                break;
            case 021: // or.16
                stage.executeOp = ExecuteOr;
                break;
            case 022: // xor.16
                stage.executeOp = ExecuteXor;
                break;
            case 023: // nor.16
                stage.executeOp = ExecuteNor;
                break;

            case 030: // mov.16 S->D
                bRegBank = 040;
                isPrivileged = true;
                break;
            case 031: // mov.16 D->S
                aRegBank = 040;
                isPrivileged = true;
                break;
            case 032: // mov.32 S->D
                isDouble = true;
                bRegBank = 040;
                isPrivileged = true;
                break;
            case 033: // mov.32 D->S
                isDouble = true;
                aRegBank = 040;
                isPrivileged = true;
                break;

            case 042: // mov.32 A->D
                isDouble = true;
            case 040: // mov.16 A->D
                bRegBank = 020;
                break;
            case 043: // mov.32 D->A
                isDouble = true;
            case 041: // mov.16 D->A
                aRegBank = 020;
                break;
            case 045: // xch.32 D<>A
                isDouble = true;
            case 044: // xch.16 D<>A
                aRegBank = 020;
                isExchange = true;
                break;

            case 050: // syscall
                break;
            case 051: // eret
            case 052: // eret D
            case 053: // eret A
                isPrivileged = true;
                break;
            case 072: // jalr D
            case 073: // jalr A
                stage.regVals[0] = stage.altInstructionPointer & 0xff;
                stage.regVals[1] = stage.altInstructionPointer >> 16;
                stage.dstRegs[0] = 016;
                stage.dstRegs[1] = 017;
            case 070: // jr D
            case 071: // jr A
                if (func == 070 || func == 071) {
                    stage.commitOp = CommitNop;
                }

                aReg <<= 1;
                if (func & 1) {
                    aReg += 020;
                }

                target = registerFile[aReg + 1];
                target <<= 16;
                target |= registerFile[aReg];

                stage.nextInstructionPointer = target;
                stage.srcRegs[0] = aReg;
                stage.srcRegs[1] = aReg + 1;

                stage.flushOnCommit = true;

                isCustom = true;
                break;
            default:
                stage.exception = true;
                break;
        }

        if (!isCustom) {
            if (isDouble) {
                aReg <<= 1;
                bReg <<= 1;

                aReg += aRegBank;
                bReg += bRegBank;

                stage.regVals[1] = registerFile[aReg + 1];
                stage.regVals[3] = registerFile[bReg + 1];
                stage.srcRegs[1] = aReg + 1;
                stage.srcRegs[3] = bReg + 1;
                stage.dstRegs[1] = aReg + 1;

                if (isExchange) {
                    stage.dstRegs[3] = bReg + 1;
                }
            }
            else {
                aReg += aRegBank;
                bReg += bRegBank;
            }

            stage.regVals[0] = registerFile[aReg];
            stage.regVals[2] = registerFile[bReg];
            stage.srcRegs[0] = aReg;
            stage.srcRegs[2] = bReg;
            stage.dstRegs[0] = aReg;

            if (isExchange) {
                stage.dstRegs[2] = bReg;
            }
        }
    }
    else if (opcode == 007 || opcode == 017) {
        // J

        stage.executeOp = ExecuteNop;
        stage.memoryOp = MemoryNop;

        if (opcode == 007) {
            stage.regVals[0] = stage.altInstructionPointer & 0xff;
            stage.regVals[1] = stage.altInstructionPointer >> 16;
            stage.dstRegs[0] = 016;
            stage.dstRegs[1] = 017;
            stage.commitOp = CommitWriteBack;
        }
    }
    else if (opcode == 001 || (opcode & 010 == 010)) { // 017 is covered by the earlier check
        // I

        uint8_t aReg = (instruction >> 9) & 0007;
        uint8_t func = (instruction >> 8) & 0001;
        uint8_t imm  =  instruction       & 0377;

        stage.memoryOp = MemoryNop;
        stage.commitOp = CommitWriteBack;

        bool signExtend = false;

        stage.srcRegs[0] = aReg;
        stage.dstRegs[0] = aReg;
        stage.regVals[2] = imm;

        switch (opcode) {
            case 001:
                aReg <<= 1;
                stage.srcRegs[0] = aReg;
                stage.srcRegs[1] = aReg + 1;
                stage.dstRegs[0] = aReg;
                stage.dstRegs[1] = aReg + 1;

                stage.regVals[1] = registerFile[aReg + 1];

                stage.executeOp = func ? ExecuteSubtract : ExecuteAdd;
                break;
            case 010:
                stage.executeOp = ExecuteAdd;
                if (!func) {
                    signExtend = true;
                }
                break;
            case 011:
                stage.executeOp = ExecuteSubtract;
                if (!func) {
                    signExtend = true;
                }
                break;
            case 012: // lui, lli
                stage.executeOp = func ? ExecuteLoadLowerImmediate : ExecuteLoadUpperImmediate;
                break;
            case 013: // and, or
                stage.executeOp = func ? ExecuteOr : ExecuteAnd;
                break;
            case 014: // xor, lsh
                stage.executeOp = func ? ExecuteLeftShift : ExecuteXor;
                break;
            case 015: // lsha, rsh
                stage.executeOp = func ? ExecuteRightShift : ExecuteRightShiftArithmetic;
                break;
            case 016: // slt
                stage.executeOp = ExecuteSetLessThan;
                if (!func) {
                    signExtend = true;
                }
                break;
            default:
                // This should be impossible to hit...
                break;
        }

        if (signExtend && (stage.regVals[2] & 0x0080)) {
            stage.regVals[2] |= 0xff00;
            stage.regVals[3]  = 0xffff;
        }
        stage.regVals[0] = registerFile[stage.srcRegs[0]];
    }
    else if (opcode == 002 || opcode == 003) {
        // M/E
        uint8_t aReg =  (instruction >> 9) & 07;
        uint8_t bReg = ((instruction >> 6) & 07) << 1;

        uint8_t func;

        if (opcode == 002) {
            func = instruction & 077;
            stage.regVals[2] = stage.regVals[1];
        }
        else {
            func = instruction & 07;

            uint8_t cReg = (instruction >> 3) & 07;
            stage.regVals[2] = registerFile[cReg];
            stage.srcRegs[3] = cReg;
        }

        stage.regVals[0] = registerFile[bReg];
        stage.regVals[1] = registerFile[bReg + 1];
        stage.srcRegs[0] = bReg;
        stage.srcRegs[1] = bReg + 1;

        stage.executeOp = ExecuteAdd;
        if (stage.regVals[2] & 0x8000) {
            stage.regVals[3] = 0xffff;
        }

        switch (func) {
            case 0: // lb
                stage.memoryOp = MemoryReadByte;
                stage.commitOp = CommitWriteBack;
                stage.dstRegs[0] = aReg;
                break;
            case 1: // lw
                stage.memoryOp = MemoryReadWord;
                stage.commitOp = CommitWriteBack;
                stage.dstRegs[0] = aReg;
                break;
            case 2: // sb
                stage.memoryOp = MemoryWriteByte;
                stage.commitOp = CommitWrite;
                stage.regVals[4] = registerFile[aReg];
                stage.srcRegs[2] = aReg;
                break;
            case 3: // sw
                stage.memoryOp = MemoryWriteWord;
                stage.commitOp = CommitWrite;
                stage.regVals[4] = registerFile[aReg];
                stage.srcRegs[2] = aReg;
                break;
            default:
                stage.exception = true;
                break;
        }
    }
    else if (opcode == 006) {
        // B
        uint32_t offset = stage.regVals[1];
        if (offset & 0x8000) {
            offset |= 0xffff0000;
        }
        stage.altInstructionPointer = stage.instructionPointer + offset;

        uint8_t aReg = (instruction >> 9) & 07;
        uint8_t bReg = (instruction >> 6) & 07;

        uint8_t func = instruction & 077;

        stage.regVals[0] = registerFile[aReg];
        stage.regVals[1] = 0;
        stage.regVals[2] = registerFile[bReg];
        stage.regVals[3] = 0;

        stage.srcRegs[0] = aReg;
        stage.srcRegs[1] = bReg;

        stage.executeOp = ExecuteSubtract;
        stage.memoryOp = MemoryNop;

        switch (func) {
            case 0:
                stage.commitOp = CommitDecideEQ;
                break;
            case 1:
                stage.commitOp = CommitDecideNE;
                break;
            case 2:
                stage.commitOp = CommitDecideGT;
                break;
            case 3:
                stage.commitOp = CommitDecideLE;
                break;
            case 4:
                stage.commitOp = CommitDecideLT;
                break;
            case 5:
                stage.commitOp = CommitDecideGE;
                break;
            default:
                stage.exception = true;
                break;
        }
    }
    else {
        // Invalid instruction opcode
        stage.exception = true;
    }

    stage.privileged = isPrivileged;

    stage.delayed = false;
    // Check register hazards
    for (int i = 0; i < 6 && !stage.delayed; i++) {
        if (stage.srcRegs[i] == StageRegister::emptyRegister) {
            continue;
        }
        if (registerHazards.contains(stage.srcRegs[i])) {
            stage.delayed = true;
            break;
        }
    }

    if (stage.delayed) {
        // unset dstRegs so they don't unintentionally impede
        // this instruction next cycle
        for (int i = 0; i < 4; i++) {
            stage.dstRegs[i] = StageRegister::emptyRegister;
        }
    }

    stageRegisters[1] = stage;
}
void N16R::executeStage() {
    if (stageRegisters.size() < 3 || stageRegisters[2].bubble) {
        return;
    }

    auto stage = stageRegisters[2];

    uint32_t temp;
    uint32_t temp0;

    switch (stage.executeOp) {
        case ExecuteSetLessThan:
        case ExecuteSubtract:
        case ExecuteAdd:
            temp  = ((uint32_t) stage.regVals[0] << 16) | stage.regVals[1];

            temp0 = ((uint32_t) stage.regVals[2] << 16) | stage.regVals[3];
            if (stage.executeOp != ExecuteAdd) {
                temp0 = ~temp0 + 1;
            }
            temp  = temp + temp0;

            if (stage.executeOp == ExecuteSetLessThan) {
                stage.regVals[0] = (temp & 0x80000000) ? 1 : 0;
            }
            else {
                stage.regVals[0] = temp >> 16;
                stage.regVals[1] = temp & 0xff;
            }

            if (stage.executeCanOverflow) {
                //
            }
            
            break;
        case ExecuteAnd:
            stage.regVals[0] &= stage.regVals[2];
            break;
        case ExecuteOr:
            stage.regVals[0] |= stage.regVals[2];
            break;
        case ExecuteNor:
            stage.regVals[0] = ~(stage.regVals[0] | stage.regVals[2]);
            break;
        case ExecuteXor:
            stage.regVals[0] ^= stage.regVals[2];
            break;
        case ExecuteLoadUpperImmediate:
            stage.regVals[0] = stage.regVals[2] << 8;
            break;
        case ExecuteLoadLowerImmediate:
            stage.regVals[0] = stage.regVals[2] & 0xff;
            break;
        case ExecuteLeftShift:
            stage.regVals[0] <<= ((stage.regVals[2] & 0xf) + 1);
            break;
        case ExecuteRightShiftArithmetic:
            temp = stage.regVals[0];
            if (temp & 0x8000) {
                temp |= 0xffff0000;
            }
            stage.regVals[0] = (temp >> ((stage.regVals[2] & 0xf) + 1)) & 0xffff;
            break;
        case ExecuteRightShift:
            stage.regVals[0] >>= ((stage.regVals[2] & 0xf) + 1);
            break;
        case ExecuteExchange:
            temp = stage.regVals[0];
            stage.regVals[0] = stage.regVals[2];
            stage.regVals[2] = temp;

            temp = stage.regVals[1];
            stage.regVals[1] = stage.regVals[3];
            stage.regVals[3] = temp;
            break;
        case ExecuteNop:
        default:
            break;
    }

    stageRegisters[2] = stage;
}
void N16R::memoryStage() {
    if (stageRegisters.size() < 4 || stageRegisters[3].bubble) {
        return;
    }

    auto stage = stageRegisters[3];

    uint32_t asid = ((uint32_t) registerFile[39] << 16) | registerFile[38];
    uint32_t memoryAddress = ((uint32_t) stage.regVals[1] << 16) | stage.regVals[0];
    uint32_t wordAddress = memoryAddress & 0xfffffffe;
    uint16_t memoryValue = stage.regVals[4];

    if (stage.memoryOp == MemoryReadByte || stage.memoryOp == MemoryReadWord) {
        if (cacheController.contains(DataCache, wordAddress, asid)) {
            memoryValue = cacheController.read(DataCache, wordAddress, asid);
            if (stage.memoryOp == MemoryReadByte) {
                if (memoryAddress & 1) {
                    memoryValue >>= 8;
                }
                else {
                    memoryValue &= 0xff;
                }
            }
            stage.regVals[0] = memoryValue;
            stage.delayed = false;
        }
        else {
            cacheController.queueRead(DataRead, wordAddress, asid);
            stage.delayed = true;
        }
    }
    else {
        MemoryOperation writeOp;
        writeOp.address = memoryAddress;
        writeOp.asid = asid;
        writeOp.type = MemoryOperationDataWrite;

        writeOp.data.push_back(stage.regVals[4] & 0xff);
        writeOp.bytes = 1;
        if (stage.memoryOp == MemoryWriteWord) {
            writeOp.bytes++;
            writeOp.data.push_back(stage.regVals[4] >> 8);
        }
        stage.regVals[1] = cacheController.queueOperation(writeOp);
    }

    stageRegisters[3] = stage;
}
void N16R::writeBackStage() {
    if (stageRegisters.size() < 5 || stageRegisters[4].bubble) {
        return;
    }

    auto stage = stageRegisters[4];

    if (stage.commitOp == CommitNop) {
        return;
    }

    if (stage.commitOp == CommitWriteBack) {
        for (int i = 0; i < 4; i++) {
            if (stage.dstRegs[i] != StageRegister::emptyRegister) {
                registerFile[stage.dstRegs[i]] = stage.regVals[i];
            }
        }
    }
    else if (stage.commitOp == CommitWrite) {
        cacheController.commitOperation(stage.regVals[1]);
    }
    else {
        auto checkValue = stage.regVals[0];

        bool zero     =  checkValue == 0;
        bool negative = (checkValue & 0x8000) != 0;

        bool taken = false;
        switch (stage.commitOp) {
            case CommitDecideEQ:
                taken = zero;
                break;
            case CommitDecideNE:
                taken = !zero;
                break;
            case CommitDecideGT:
                taken = !zero && !negative;
                break;
            case CommitDecideLE:
                taken = zero || negative;
                break;
            case CommitDecideLT:
                taken = negative;
                break;
            case CommitDecideGE:
                taken = !negative;
                break;
            default:
                taken = false;
                break;
        }

        if (taken) {
            stage.nextInstructionPointer = stage.altInstructionPointer;
            stageFlush(4);
        }
    }
    stageRegisters[4] = stage;
}

void N16R::stageFlush(int until) {
    for (int i = 0; i < 5 && i < until; i++) {
        stageRegisters[i].invalidate();
        if (i == 3 && stageRegisters[i].commitOp == CommitWrite) {
            cacheController.invalidateOperation(stageRegisters[i].regVals[1]);
        }
    }
}

void N16R::stageShift() {
    auto justRetired = stageRegisters.back();
    if (stageRegisters.size() == 5) {
        stageRegisters[4].invalidate();
    }

    for (int i = stageRegisters.size() - 2; i >= 0; i++) {
        // check if we can progress forward
        if (stageRegisters[i+1].bubble) {
            // and check if we're delayed
            if (!stageRegisters[i].delayed) {
                stageRegisters[i+1] = stageRegisters[i];
                stageRegisters[i].invalidate();
            }
        }
    }

    int inFlight = 0;
    registerHazards.clear();

    for (auto stage: stageRegisters) {
        if (!stage.bubble) {
            inFlight++;

            for (int i = 0; i < 4; i++) {
                if (stage.dstRegs[i] != StageRegister::emptyRegister) {
                    registerHazards.insert(stage.dstRegs[i]);
                }
            }
        }
    }

    if (stageRegisters[0].bubble) {
        StageRegister nextStart;
        if (isPipelined) {
            auto lastFetched = stageRegisters[1];

            nextStart.instructionPointer = lastFetched.nextInstructionPointer;
            nextStart.nextInstructionPointer = nextStart.instructionPointer;
            nextStart.bubble = false;
        }
        else if (inFlight == 0) {
            nextStart.instructionPointer = justRetired.nextInstructionPointer;
            nextStart.nextInstructionPointer = nextStart.instructionPointer;
            nextStart.bubble = false;
        }

        stageRegisters[0] = nextStart;
    }
}

/*
{

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
                    case 052: // eret r32
                    case 053: // eret a32
                        if (!isKernel()) {
                            pendingException = ExceptionType::ReservedInstruction;
                            break;
                        }
                        // shift the kernel state stack
                        tmp = sysRegisterFile[1] & 0xfff0 | ((sysRegisterFile[1] >> 2) & 0xf);
                        sysRegisterFile[1] = tmp;
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
            else if (decoder.format == InstructionFormat::R && (decoder.function == 052 || decoder.function == 053)) {
                // start with a simple eret d.32
                uint16_t rl = registerFile[ decoder.dReg << 1     ];
                uint16_t rh = registerFile[(decoder.dReg << 1) + 1];

                // oops, no it's an eret a.32
                if (decoder.function == 053) {
                    rl = altRegisterFile[ decoder.dReg << 1     ];
                    rh = altRegisterFile[(decoder.dReg << 1) + 1];
                }

                instructionPointer = ((uint32_t) rh) << 16 | rl;
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
    if (sysRegisterFile[1] & 1) {
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
*/

bool N16R::isKernel() {
    return (registerFile[041] & 0x2) == 0;
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
            response << "1" << i << ": " << std::setw(4) << std::setfill('0') << std::hex << registerFile[i + 020];
            response << "       ";
            response << "2" << i << ": " << std::setw(4) << std::setfill('0') << std::hex << registerFile[i + 040];
            response << std::endl;
        }
        for (int i = 0; i < 8; i++) {
            int e = i << 1;
            uint32_t r =    registerFile[e + 001];
            r = (r << 16) | registerFile[e + 000];
            response << "4" << i << ": " << std::setw(8) << std::setfill('0') << std::hex << r << "   ";
            r =             registerFile[e + 021];
            r = (r << 16) | registerFile[e + 020];
            response << "5" << i << ": " << std::setw(8) << std::setfill('0') << std::hex << r << "   ";
            r =             registerFile[e + 041];
            r = (r << 16) | registerFile[e + 040];
            response << "6" << i << ": " << std::setw(8) << std::setfill('0') << std::hex << r;
            response << std::endl;
        }
        //executionBuffer.output(response);
        response << "Ok.";
    }
    else {
        response << "Ok.";
    }
    return response.str();
}

void N16R::reset() {
    StageRegister resetVector;
    resetVector.instructionPointer = resetAddress;
    resetVector.nextInstructionPointer = resetAddress;
    resetVector.bubble = false;

    stageRegisters.clear();
    stageRegisters.insert(stageRegisters.begin(), resetVector);

    registerFile[041] = 0;
    //executionBuffer.reset();
    busUnit.reset();
}

StageRegister::StageRegister(): executeOp(ExecuteNop), memoryOp(MemoryNop), commitOp(CommitNop) {
    invalidate();
}

void StageRegister::invalidate() {
    bubble = true;
    exception = false;
    delayed = false;
    for (int i = 0; i < 6; i++) {
        regVals[i] = 0;
        srcRegs[i] = emptyRegister;
    }
    for (int i = 0; i < 4; i++) {
        dstRegs[i] = emptyRegister;
    }
}

}; // namespace n16r

}; // namespace nbus

}; // namespace sysnp


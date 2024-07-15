#include "n16r.h"
#include <iostream>
#include <iomanip>

namespace sysnp {

namespace nbus {

namespace n16r {

uint16_t byteswap(uint16_t val) {
    uint16_t ret = val << 8;
    ret |= val >> 8;
    return ret;
}
uint16_t e8s16(uint8_t in) {
    uint16_t out = in;
    if (out & 0x80) {
        out |= 0xff00;
    }
    return out;
}
uint32_t e16s32(uint16_t in) {
    uint32_t out = in;
    if (out & 0x8000) {
        out |= 0xffff0000;
    }
    return out;
}

void N16R::init(const libconfig::Setting &setting) {
    isPipelined = false;
    setting.lookupValue("resetAddress", resetAddress);
    setting.lookupValue("pipelined", isPipelined);

    const libconfig::Setting& cCacheC = setting["cache"];
    const libconfig::Setting& cCaches = cCacheC["caches"];
    const libconfig::Setting& cNoCache = cCacheC["noCache"];

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

    int cacheCount = cCaches.getLength();
    for (int i = 0; i < cacheCount; i++) {
        const libconfig::Setting& cCache = cCaches[i];
        std::string type;
        int binBits = 0;
        int lineBits = 0;
        int ways = 0;
        cCache.lookupValue("type", type);
        cCache.lookupValue("binBits", binBits);
        cCache.lookupValue("lineBits", lineBits);
        cCache.lookupValue("ways", ways);
        cacheController.setCache(
            type == "data" ? DataCache : InstructionCache,
            32, // address bits
            binBits,
            lineBits,
            ways
        );
    }

    registerFile[040] = 0;
    registerFile[041] = 0;
}

void N16R::postInit() {
    machine->debug("N16R::postInit()");
    busUnit.setBusInterface(interface);
    reset();

    lastBreakpoint = 0;
    breakpointWasHit = false;

    retiredAddresses.set_capacity(512);
}

void N16R::clockUp() {
    machine->debug("N16R::clockUp()");

    writeBackStage();
    memoryStage();
    executeStage();
    decodeStage();
    fetchStage();

    // Bus interface
    machine->debug("Processing bus unit");

    if (busUnit.isIdle()) {
        if (cacheController.isOperationPrepared()) {
            machine->debug("Queueing operation");
            busUnit.queueOperation(cacheController.getBusOperation());
        }
    }
    busUnit.clockUp();
}

void N16R::clockDown() {
    machine->debug("N16R::clockDown()");

    machine->debug("Shifting stages");
    stageShift();
    stageClearOut();

    clockCount++;

    machine->debug("Processing bus unit");
    busUnit.clockDown();

    if (busUnit.hasData()) {
        cacheController.ingestWord(busUnit.getWord());
    }

    uint16_t pendingInterrupts = busUnit.hasInterrupt();
    pendingInterrupts <<= 8;
    pendingInterrupts &= registerFile[statusRegister];

    registerFile[causeRegister] = (registerFile[causeRegister] & 0xff) | pendingInterrupts;
}

void N16R::fetchStage() {
    if (stageRegisters[0].bubble) {
        machine->debug(7, "Fetch bubble.");
        return;
    }

    StageRegister stage = stageRegisters[0];

    uint32_t asid = 0;

    if (breakpoints.contains(stage.instructionPointer)) {
        if (stage.instructionPointer != lastBreakpoint) {
            lastBreakpoint = stage.instructionPointer;
            breakpointWasHit = true;
        }
        else {
            breakpointWasHit = false;
        }
    }
    else {
        lastBreakpoint = 0;
        breakpointWasHit = false;
    }

    uint32_t nextWord = stage.instructionPointer + 2;

    if (stage.delayed && stage.instructionPointer != stage.nextInstructionPointer) {
        machine->debug("Fetch pre-delayed");

        if (cacheController.contains(InstructionCache, nextWord, 2, asid) != CacheContainsNone) {
            stage.fetch[1] = byteswap(cacheController.read(InstructionCache, nextWord, 2, asid));
            stage.delayed = false;
        }
        else {
            cacheController.queueRead(InstructionRead, nextWord, 2, asid);
            stage.delayed = true;
        }
    }
    else if (cacheController.contains(InstructionCache, stage.instructionPointer, 2, asid) != CacheContainsNone) {
        machine->debug("Fetch found");
        stage.fetch[0] = byteswap(cacheController.read(InstructionCache, stage.instructionPointer, 2, asid));
        stage.delayed = false;
    }
    else if (!stage.delayed) {
        machine->debug("Fetch queued");
        cacheController.queueRead(InstructionRead, stage.instructionPointer, 2, asid);
        stage.delayed = true;
    }
    else {
        machine->debug("Waiting for instruction data");
    }

    if (!stage.delayed) {
        // early-decode
        uint8_t opcode = (stage.fetch[0] >> 12) & 0xf;
        if (opcode == 002 || opcode == 006 || opcode == 007 || opcode == 017) {
            stage.nextInstructionPointer = stage.instructionPointer + 4;
            stage.altInstructionPointer = stage.nextInstructionPointer;

            if (cacheController.contains(InstructionCache, nextWord, 2, asid) != CacheContainsNone) {
                stage.fetch[1] = byteswap(cacheController.read(InstructionCache, nextWord, 2, asid));
            }
            else {
                cacheController.queueRead(InstructionRead, nextWord, 2, asid);
                stage.delayed = true;
            }
        }
        else {
            stage.nextInstructionPointer = stage.instructionPointer + 2;
            stage.altInstructionPointer = stage.nextInstructionPointer;
        }

        if (!stage.delayed && (opcode == 007 || opcode == 017)) {
            uint32_t baseI = stage.fetch[0] & 0xfff;
            uint32_t extrI = stage.fetch[1];
            stage.altInstructionPointer = stage.instructionPointer & 0xe0000000 | (baseI << 17) | (extrI << 1);
            stage.nextInstructionPointer = stage.altInstructionPointer;
        }
    }

    stageRegisters[0] = stage;
}

void N16R::decodeStage() {
    if (stageRegisters[1].bubble) {
        machine->debug(7, "Decode is bubble.");
        return;
    }
    if (stageRegisters[1].exception) {
        machine->debug(7, "Decode is exception.");
        return;
    }

    auto stage = stageRegisters[1];

    stage.commitOp = CommitNop;
    stage.executeOp = ExecuteNop;

    auto instruction = stage.fetch[0];
    uint8_t opcode = (instruction >> 12) & 0xf;

    uint32_t target;

    bool latePopulateNext = false;
    bool isPrivileged = false;

    if (opcode == 000) {
        // R

        uint8_t aReg = (instruction >> 9) & 07;
        uint8_t bReg = (instruction >> 6) & 07;
        uint8_t func = instruction & 077;

        stage.executeCanOverflow = false;
        stage.commitOp = CommitWriteBack;

        stage.exceptionType = ExceptNone;

        bool isDouble = false;
        bool isCustom = false;
        bool aRegOnly = false;

        uint8_t aRegBank = 0;
        uint8_t bRegBank = 0;

        bool isExchange = false;


        switch (func) {
            case 001: // mov.32 D->D
                isDouble = true;
            case 000: // mov.16 D->D
                stage.executeOp = ExecutePickB;
                break;
            case 003: // xch.32 D<>D
                isDouble = true;
            case 002: // xch.16 D<>D
                stage.executeOp = ExecuteExchange;
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
                stage.executeOp = ExecutePickB;
                bRegBank = 040;
                isPrivileged = true;
                break;
            case 031: // mov.16 D->S
                stage.executeOp = ExecutePickB;
                aRegBank = 040;
                isPrivileged = true;
                break;
            case 032: // mov.32 S->D
                stage.executeOp = ExecutePickB;
                isDouble = true;
                bRegBank = 040;
                isPrivileged = true;
                break;
            case 033: // mov.32 D->S
                stage.executeOp = ExecutePickB;
                isDouble = true;
                aRegBank = 040;
                isPrivileged = true;
                break;

            case 042: // mov.32 A->D
                isDouble = true;
            case 040: // mov.16 A->D
                stage.executeOp = ExecutePickB;
                bRegBank = 020;
                break;
            case 043: // mov.32 D->A
                isDouble = true;
            case 041: // mov.16 D->A
                stage.executeOp = ExecutePickB;
                aRegBank = 020;
                break;
            case 045: // xch.32 D<>A
                isDouble = true;
            case 044: // xch.16 D<>A
                stage.executeOp = ExecuteExchange;
                aRegBank = 020;
                isExchange = true;
                break;

            case 050: // syscall
                stage.commitOp = CommitSyscall;
                break;
            case 051: // eret
                stage.commitOp = CommitExceptionReturn;
                isPrivileged = true;
                break;
            case 053: // eret A
                aRegBank = 020;
            case 052: // eret D
                stage.commitOp = CommitExceptionReturnJump;
                isPrivileged = true;
                isDouble = true;
                aRegOnly = true;
                break;

            case 072: // jalr D
            case 073: // jalr A
                stage.decode[0] = (stage.instructionPointer + 2) & 0xff;
                stage.decode[1] = (stage.instructionPointer + 2) >> 16;
                stage.dstRegs[0] = 016;
                stage.dstRegs[1] = 017;
            case 070: // jr D
            case 071: // jr A
                if (func == 070 || func == 071) {
                    stage.commitOp = CommitJump;
                }

                aReg <<= 1;
                if (func & 1) {
                    aReg += 020;
                }

                stage.decode[2] = registerFile[aReg];
                stage.decode[3] = registerFile[aReg + 1];
                stage.srcRegs[2] = aReg;
                stage.srcRegs[3] = aReg + 1;

                latePopulateNext = true;

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

                stage.decode[1] = registerFile[aReg + 1];
                stage.decode[3] = registerFile[bReg + 1];
                stage.srcRegs[1] = aReg + 1;

                if (!aRegOnly) {
                    stage.srcRegs[3] = bReg + 1;
                    stage.dstRegs[1] = aReg + 1;

                    if (isExchange) {
                        stage.dstRegs[3] = bReg + 1;
                    }
                }
            }
            else {
                aReg += aRegBank;
                bReg += bRegBank;
            }

            stage.decode[0] = registerFile[aReg];
            stage.decode[2] = registerFile[bReg];
            stage.srcRegs[0] = aReg;

            if (!aRegOnly) {
                stage.srcRegs[2] = bReg;
                stage.dstRegs[0] = aReg;

                if (isExchange) {
                    stage.dstRegs[2] = bReg;
                }
            }
        }
    }
    else if (opcode == 007 || opcode == 017) {
        // J

        stage.executeOp = ExecuteAdd;
        stage.memoryOp = MemoryNop;

        if (opcode == 007) {
            stage.decode[0] = (stage.instructionPointer + 4) & 0xffff;
            stage.decode[1] = (stage.instructionPointer + 4) >> 16;
            stage.dstRegs[0] = 016;
            stage.dstRegs[1] = 017;
            stage.commitOp = CommitWriteBack;
        }
    }
    else if (opcode == 001 || ((opcode & 010) == 010)) { // 017 is covered by the earlier check
        // I

        uint8_t aReg = (instruction >> 9) & 0007;
        uint8_t func = (instruction >> 8) & 0001;
        uint8_t imm  =  instruction       & 0377;

        stage.memoryOp = MemoryNop;
        stage.commitOp = CommitWriteBack;

        bool signExtend = false;

        stage.srcRegs[0] = aReg;
        stage.dstRegs[0] = aReg;
        stage.decode[2] = imm;

        switch (opcode) {
            case 001:
                aReg <<= 1;
                stage.srcRegs[0] = aReg;
                stage.srcRegs[1] = aReg + 1;
                stage.dstRegs[0] = aReg;
                stage.dstRegs[1] = aReg + 1;

                stage.decode[1] = registerFile[stage.srcRegs[1]];

                stage.executeOp = func ? ExecuteSubtractHalf : ExecuteAddHalf;
                break;
            case 010:
                stage.executeOp = ExecuteAddHalf;
                if (!func) {
                    signExtend = true;
                }
                break;
            case 011:
                stage.executeOp = ExecuteSubtractHalf;
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

        if (signExtend) {
            stage.decode[2] = e8s16(stage.decode[2]);
        }
        stage.decode[0] = registerFile[stage.srcRegs[0]];
    }
    else if (opcode == 002 || opcode == 003) {
        // M/E
        uint8_t aReg =  (instruction >> 9) & 07;
        uint8_t bReg = ((instruction >> 6) & 07) << 1;

        uint8_t func;

        if (opcode == 002) {
            func = instruction & 077;
            stage.decode[2] = stage.fetch[1];
        }
        else {
            func = instruction & 07;

            uint8_t cReg = (instruction >> 3) & 07;
            stage.decode[2] = registerFile[cReg];
            stage.srcRegs[2] = cReg;
        }

        stage.decode[0] = registerFile[bReg];
        stage.decode[1] = registerFile[bReg + 1];
        stage.srcRegs[0] = bReg;
        stage.srcRegs[1] = bReg + 1;

        stage.executeOp = ExecuteAddHalf;

        switch (func) {
            case 2: // ld
                aReg <<= 1;
                stage.dstRegs[6] = aReg + 1;
            case 0: // lb
            case 1: // lw
                stage.memoryOp = MemoryRead;
                stage.memoryBytes = func == 0 ? 1 : (func == 1 ? 2 : 4);
                stage.commitOp = CommitWriteBack;
                stage.dstRegs[5] = aReg;
                break;
            case 6: // sd
                aReg <<= 1;
                stage.decode[4] = registerFile[aReg + 1];
                stage.srcRegs[4] = aReg + 1;
            case 4: // sb
            case 5: // sw
                stage.memoryOp = MemoryWrite;
                stage.memoryBytes = func == 4 ? 1 : (func == 5 ? 2 : 4);
                stage.commitOp = CommitWrite;
                stage.decode[3] = registerFile[aReg];
                stage.srcRegs[3] = aReg;
                break;
            default:
                stage.exception = true;
                break;
        }
    }
    else if (opcode == 006) {
        // B
        uint32_t offset = e16s32(stage.fetch[1]);
        stage.altInstructionPointer = stage.instructionPointer + (offset << 1);

        uint8_t aReg = (instruction >> 9) & 07;
        uint8_t bReg = (instruction >> 6) & 07;

        uint8_t func = instruction & 077;

        stage.decode[0] = registerFile[aReg];
        stage.decode[1] = 0;
        stage.decode[2] = registerFile[bReg];
        stage.decode[3] = 0;

        stage.srcRegs[0] = aReg;
        stage.srcRegs[2] = bReg;

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

    if (stage.exception && stage.exceptionType == ExceptNone) {
        stage.exceptionType = ExceptInvalidInstruction;
std::cout << "invalid" << std::endl;
    }

    stage.privileged = isPrivileged;

    stage.delayed = false;
    // Check register hazards
    for (int i = 0; i < 5; i++) {
        if (stage.srcRegs[i] == StageRegister::emptyRegister) {
            continue;
        }
        for (int s = 2; s < 5; s++) {
            OperandHazard hazard = stageRegisters[s].checkOperandHazard(stage.srcRegs[i], s);
            if (hazard == OperandHazardNone) {
                continue;
            }
            if (hazard == OperandHazardNext) {
                stage.delayed = true;
                break;
            }
            stage.decode[i] = stageRegisters[s].operandForward(stage.srcRegs[i], s);
            break;
        }
        if (stage.delayed) {
            break;
        }
    }

    if (latePopulateNext) {
        target = stage.decode[2];
        target |= ((uint32_t) stage.decode[3]) << 16;

        stage.nextInstructionPointer = target;
        stage.altInstructionPointer = target;
    }

    stageRegisters[1] = stage;
}

void N16R::executeStage() {
    if (stageRegisters[2].bubble) {
        machine->debug(7, "Execute is bubble.");
        return;
    }
    if (stageRegisters[2].exception) {
        machine->debug(7, "Execute is exception.");
        return;
    }

    auto stage = stageRegisters[2];

    uint32_t temp;
    uint32_t temp0;

    switch (stage.executeOp) {
        case ExecuteSetLessThan:
        case ExecuteSubtract:
        case ExecuteSubtractHalf:
        case ExecuteAdd:
        case ExecuteAddHalf:
            temp  = ((uint32_t) stage.decode[1] << 16) | stage.decode[0];

            if (stage.executeOp == ExecuteAddHalf || stage.executeOp == ExecuteSubtractHalf) {
                temp0 = e16s32(stage.decode[2]);
            }
            else {
                temp0 = ((uint32_t) stage.decode[3] << 16) | stage.decode[2];
            }

            if (stage.executeOp != ExecuteAdd && stage.executeOp != ExecuteAddHalf) {
                temp0 = ~temp0 + 1;
            }
            temp  = temp + temp0;

            if (stage.executeOp == ExecuteSetLessThan) {
                stage.execute[0] = (temp & 0x80000000) ? 1 : 0;
            }
            else {
                stage.execute[0] = temp & 0xffff;
                stage.execute[1] = temp >> 16;
            }

            if (stage.executeCanOverflow) {
                //
            }
            
            break;
        case ExecuteAnd:
            stage.execute[0] = stage.decode[0] & stage.decode[2];
            break;
        case ExecuteOr:
            stage.execute[0] = stage.decode[0] | stage.decode[2];
            break;
        case ExecuteNor:
            stage.execute[0] = ~(stage.decode[0] | stage.decode[2]);
            break;
        case ExecuteXor:
            stage.execute[0] = stage.decode[0] ^ stage.decode[2];
            break;
        case ExecuteLoadUpperImmediate:
            stage.execute[0] = stage.decode[2] << 8;
            break;
        case ExecuteLoadLowerImmediate:
            stage.execute[0] = stage.decode[2] & 0xff;
            break;
        case ExecuteLeftShift:
            stage.execute[0] = stage.decode[0] << ((stage.decode[2] & 0xf) + 1);
            break;
        case ExecuteRightShiftArithmetic:
            temp = e16s32(stage.decode[0]);
            stage.execute[0] = (temp >> ((stage.decode[2] & 0xf) + 1)) & 0xffff;
            break;
        case ExecuteRightShift:
            stage.execute[0] = stage.decode[0] >> ((stage.decode[2] & 0xf) + 1);
            break;
        case ExecuteExchange:
            stage.execute[2] = stage.decode[0];
            stage.execute[3] = stage.decode[1];
            // fall through
        case ExecutePickB:
            stage.execute[0] = stage.decode[2];
            stage.execute[1] = stage.decode[3];
            break;
        case ExecuteNop: // just pass them through
            stage.execute[0] = stage.decode[0];
            stage.execute[1] = stage.decode[1];
        default:
            break;
    }

    stageRegisters[2] = stage;
}

void N16R::memoryStage() {
    if (stageRegisters[3].bubble) {
        machine->debug(7, "Memory is bubble.");
        return;
    }
    if (stageRegisters[3].exception) {
        machine->debug(7, "Memory is exception.");
        return;
    }

    auto stage = stageRegisters[3];

    if (stage.memoryOp == MemoryNop) {
        return;
    }

    uint32_t asid = ((uint32_t) registerFile[asidRegister + 1] << 16) | registerFile[asidRegister];
    uint32_t memoryAddress = ((uint32_t) stage.execute[1] << 16) | stage.execute[0];

    if (stage.memoryOp == MemoryRead) {
        CacheCheck cacheCheck = cacheController.contains(DataCache, memoryAddress, stage.memoryBytes, asid);
        if (cacheCheck == CacheContainsNone || cacheCheck == CacheContainsPartial) {
            cacheController.queueRead(DataRead, memoryAddress, stage.memoryBytes, asid);
            if (cacheCheck == CacheContainsPartial) {
                stage.memory[0] = 1;
            }
            stage.delayed = true;
        }
        else {
            if ((cacheCheck == CacheContainsSplit && stage.memory[0] > 0) || cacheCheck == CacheContainsSingle) {
                uint32_t memoryValue = cacheController.read(DataCache, memoryAddress, stage.memoryBytes, asid);

                if (stage.memoryBytes == 1) {
                    stage.memory[0] = memoryValue & 0xff;
                }
                else {
                    stage.memory[0] = memoryValue & 0xffff;
                    if (stage.memoryBytes > 2) {
                        stage.memory[1] = memoryValue >> 16;
                    }
                }

                stage.delayed = false;
            }
            else {
                // we delay one clock to access the additional cache line
                stage.memory[0] = 1;
                stage.delayed = true;
            }
        }
    }
    else {
        uint16_t memoryValue0 = stage.decode[3];
        uint16_t memoryValue1 = stage.decode[4];

        MemoryOperation writeOp;
        writeOp.address = memoryAddress;
        writeOp.asid = asid;
        writeOp.type = MemoryOperationDataWrite;

        writeOp.data.push_back(memoryValue0 & 0xff);
        writeOp.bytes = stage.memoryBytes;
        if (stage.memoryBytes > 1) {
            writeOp.data.push_back(memoryValue0 >> 8);

            if (stage.memoryBytes > 2) {
                writeOp.data.push_back(memoryValue1 & 0xff);
                writeOp.data.push_back(memoryValue1 >> 8);
            }
        }
        uint16_t pendingOpId = cacheController.queueOperation(writeOp);
        if (pendingOpId == MemoryOperation::invalidOperationId) {
            stage.delayed = true;
        }
        else {
            stage.delayed = false;
            stage.memory[0] = pendingOpId;
        }
    }

    stageRegisters[3] = stage;
}

void N16R::writeBackStage() {
    if (stageRegisters[4].bubble) {
        machine->debug(7, "Write back is bubble.");
        return;
    }

    auto stage = stageRegisters[4];

    if (hasInterrupts() && (registerFile[statusRegister] & 1)) {
        stage.exception = true;
        stage.exceptionType = ExceptInterrupt;
    }
    else if (stage.privileged && !isKernel()) {
        stage.exception = true;
        stage.exceptionType = ExceptReservedInstruction;
    }

    if (stage.commitOp == CommitSyscall) {
        stage.exception = true;
        stage.exceptionType = ExceptSyscall;
    }

    if (stage.exception) {
        uint32_t ipc = registerFile[ipcRegister];
        ipc |= (registerFile[ipcRegister + 1] << 16);
        stage.nextInstructionPointer = ipc;
        stage.altInstructionPointer = ipc;
        stageRegisters[4] = stage;

        uint16_t cause  = registerFile[causeRegister ];
        uint16_t status = registerFile[statusRegister];

        cause  = (cause  & 0xff00) | (stage.exceptionType << 2);
        status = (status & 0xff00) | ((status << 2) & 0x003c);

        // store cause and status
        registerFile[causeRegister ] = cause;
        registerFile[statusRegister] = status;

        // store bad virtual address (later)
        //registerFile[bvaRegister    ] = stage.regVals[0];
        //registerFile[bvaRegister + 1] = stage.regVals[1];

        // store excepting instruction pointer
        registerFile[epcRegister    ] = stage.instructionPointer & 0xffff;
        registerFile[epcRegister + 1] = stage.instructionPointer >> 16;

        stageFlush(4);
        if (stage.commitOp == CommitWrite) {
            cacheController.invalidateOperation(stage.memory[0]);
        }
        return;
    }

    bool taken = false;

    if (stage.commitOp == CommitNop) {
        return;
    }
    else if (stage.commitOp == CommitExceptionReturn || stage.commitOp == CommitExceptionReturnJump) {
        uint16_t status = registerFile[statusRegister];
        status = (status & 0xff00) | ((status >> 2) & 0x003f);
        registerFile[statusRegister] = status;

        if (stage.commitOp == CommitExceptionReturnJump) {
            taken = true;
            stage.altInstructionPointer = stage.execute[0];
            stage.altInstructionPointer |= (stage.execute[1] << 16);
        }
    }
    else if (stage.commitOp == CommitJump) {
        taken = true;
    }
    else if (stage.commitOp == CommitWriteBack) {
        int i = 0;
        for (; i < 5; i++) {
            if (stage.dstRegs[i] != StageRegister::emptyRegister) {
                registerFile[stage.dstRegs[i]] = stage.execute[i];
            }
        }
        for (; i < 7; i++) {
            if (stage.dstRegs[i] != StageRegister::emptyRegister) {
                registerFile[stage.dstRegs[i]] = stage.memory[i - 5];
            }
        }
    }
    else if (stage.commitOp == CommitWrite) {
        cacheController.commitOperation(stage.memory[0]);
    }
    else {
        auto checkValue = stage.execute[0];

        bool zero     =  checkValue == 0;
        bool negative = (checkValue & 0x8000) != 0;

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
    }

    stage.taken = taken;

    if (taken) {
        stageFlush(4);
    }

    stageRegisters[4] = stage;
}

void N16R::stageFlush(int until) {
    for (int i = 0; i < 5 && i < until; i++) {
        stageRegisters[i].invalidate();
        if (i >= 3 && stageRegisters[i].commitOp == CommitWrite) {
            cacheController.invalidateOperation(stageRegisters[i].memory[0]);
        }
    }
}

void N16R::stageShift() {
    auto justRetired = stageRegisters.back();

    if (!justRetired.bubble) {
        retiredAddresses.push_back(justRetired.instructionPointer);
        retiredCount++;
    }

    stageRegisters[4].invalidate();

    for (int i = stageRegisters.size() - 2; i >= 0; i--) {
        // check if we can progress forward
        if (stageRegisters[i+1].bubble) {
            // and check if we're delayed
            if (!stageRegisters[i].delayed) {
                stageRegisters[i+1] = stageRegisters[i];
                stageRegisters[i].invalidate();
            }
        }
    }

    registerHazards.clear();

    for (auto stage: stageRegisters) {
        if (!stage.bubble) {
            for (int i = 0; i < 5; i++) {
                if (stage.dstRegs[i] != StageRegister::emptyRegister) {
                    registerHazards.insert(stage.dstRegs[i]);
                }
            }
        }
    }

    if (stageRegisters[0].bubble || justRetired.exception) {
        StageRegister nextStart;

        if (justRetired.exception || justRetired.taken) {
            nextStart.instructionPointer = justRetired.altInstructionPointer;
        }
        else {
            for (auto stage: stageRegisters) {
                if (!stage.bubble) {
                    nextStart.instructionPointer = stage.nextInstructionPointer;
                    break;
                }
            }
        }
        nextStart.nextInstructionPointer = nextStart.instructionPointer;
        nextStart.altInstructionPointer  = 0xdeadbeef;
        nextStart.bubble = false;

        stageRegisters[0] = nextStart;
    }
}

void N16R::stageClearOut() {
    for (int i = 0; i < 5; i++) {
        stageRegistersOut[i].invalidate();
    }
}

bool N16R::isKernel() {
    return (registerFile[statusRegister] & 0x2) == 0;
}
bool N16R::hasInterrupts() {
    return (registerFile[causeRegister] & 0xff00) > 0;
}

std::string N16R::command(std::stringstream &input) {
    std::stringstream response;

    std::string commandWord = "status";
    input >> commandWord;

    if (commandWord == "reset") {
        reset();
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
    }
    else if (commandWord == "pipeline") {
        response << "IP         NXIP       ALIP       PDBET   INST EXTR  X  M  C   D0   D1   D2   D3   D4    X0   X1   X2   X3   X4    M0   M1" << std::endl;
        int sr = 0;
        for (auto iter = stageRegisters.cbegin(); iter != stageRegisters.cend(); iter++, sr++) {
            response << std::setw(8) << std::setfill('0') << std::hex << iter->instructionPointer << "   ";
            response << std::setw(8) << std::setfill('0') << std::hex << iter->nextInstructionPointer << "   ";
            response << std::setw(8) << std::setfill('0') << std::hex << iter->altInstructionPointer << "   ";
            response << (iter->privileged ? '1' : '0') << (iter->delayed   ? '1' : '0');
            response << (iter->bubble     ? '1' : '0') << (iter->exception ? '1' : '0');
            response << (iter->taken      ? '1' : '0') << "  ";

            if (iter->bubble) {
                response << std::endl;
                continue;
            }

            for (int i = 0; i < 2; i++) {
                if (sr > 0) {
                    response << " " << std::setw(4) << std::setfill('0') << std::hex << iter->fetch[i];
                }
                else {
                    response << "     ";
                }
            }
            response << "  ";

            if (sr > 1) {
                response << std::setw(2) << std::setfill('0') << iter->executeOp << " ";
                response << std::setw(2) << std::setfill('0') << iter->memoryOp << " ";
                response << std::setw(2) << std::setfill('0') << iter->commitOp << " ";
            }
            else {
                response << "         ";
            }

            for (int i = 0; i < 5; i++) {
                if (sr > 1) {
                    response << " " << std::setw(4) << std::setfill('0') << std::hex << iter->decode[i];
                }
                else {
                    response << "     ";
                }
            }
            response << " ";
            for (int i = 0; i < 5; i++) {
                if (sr > 2) {
                    response << " " << std::setw(4) << std::setfill('0') << std::hex << iter->execute[i];
                }
                else {
                    response << "     ";
                }
            }
            response << " ";
            for (int i = 0 ; i < 2; i++) {
                if (sr > 3) {
                    response << " " << std::setw(4) << std::setfill('0') << std::hex << iter->memory[i];
                }
                else {
                    response << "     ";
                }
            }

            if (iter->exception) {
                response << " " << iter->exceptionType;
            }
            response << std::endl;
        }
    }
    else if (commandWord == "memio") {
        response << cacheController.describeQueuedOperations() << std::endl;
    }
    else if (commandWord == "cache") {
        response << cacheController.listContents(input);
    }
    else if (commandWord == "trace") {
        int width = 0;
        for (auto address: retiredAddresses) {
            response << std::setw(8) << std::setfill('0') << std::hex << address;
            if (++width > 8) {
                response << std::endl;
                width = 0;
            }
            else {
                response << "  ";
            }
        }
        if (width != 0) {
            response << std::endl;
        }

        response << std::dec << "c: " << clockCount << ", r: " << retiredCount << ", ipc: " << ((float) retiredCount / (float) clockCount) << ", cpi: " << ((float) clockCount / (float) retiredCount) << std::endl;
    }

    response << "Ok.";

    return response.str();
}

void N16R::reset() {
    StageRegister resetVector;
    resetVector.instructionPointer = resetAddress;
    resetVector.nextInstructionPointer = resetAddress;
    resetVector.bubble = false;

    StageRegister bubble;
    bubble.bubble = true;

    stageRegisters.clear();
    stageRegistersOut.clear();

    stageRegisters.push_back(resetVector);
    stageRegistersOut.push_back(bubble);

    for (int i = 0; i < 4; i++) {
        stageRegisters.push_back(bubble);
        stageRegistersOut.push_back(bubble);
    }

    registerFile[041] = 0;
    //executionBuffer.reset();
    busUnit.reset();

    retiredAddresses.clear();
    clockCount = 0;
    retiredCount = 0;
}

bool N16R::breakpointHit() {
    bool wasHit = breakpointWasHit;
    breakpointWasHit = false;
    return wasHit;
}
void N16R::breakpointClear() {
    breakpoints.clear();
}
void N16R::breakpointAdd(uint32_t addr) {
    if (!breakpoints.contains(addr)) {
        breakpoints.insert(addr);
    }
}
void N16R::breakpointRemove(uint32_t addr) {
    auto location = breakpoints.find(addr);
    if (location != breakpoints.end()) {
        breakpoints.erase(location);
    }
}

StageRegister::StageRegister(): executeOp(ExecuteNop), memoryOp(MemoryNop), commitOp(CommitNop) {
    invalidate();
    privileged = false;
    taken = false;
}

void StageRegister::invalidate() {
    bubble = true;
    exception = false;
    delayed = false;
    fetch[0] = fetch[1] = memory[0] = memory[1] = 0;
    for (int i = 0; i < 5; i++) {
        decode[i] = execute[i] = 0;
        srcRegs[i] = emptyRegister;
    }
    for (int i = 0; i < 7; i++) {
        dstRegs[i] = emptyRegister;
    }
}

OperandHazard StageRegister::checkOperandHazard(uint8_t reg, uint8_t stage) {
    if (bubble) {
        return OperandHazardNone;
    }
    if (stage == 2) {
        for (int i = 0; i < 5; i++) {
            if (reg == dstRegs[i]) {
                return OperandHazardCurrent;
            }
        }
        if (reg == dstRegs[5] || reg == dstRegs[6]) {
            return OperandHazardNext;
        }
    }
    else if (stage >= 3) {
        for (int i = 0; i < 7; i++) {
            if (reg == dstRegs[i]) {
                return delayed ? OperandHazardNext : OperandHazardCurrent;
            }
        }
    }

    return OperandHazardNone;
}
uint16_t StageRegister::operandForward(uint8_t reg, uint8_t stage) {
    for (int i = 0; i < 5; i++) {
        if (reg == dstRegs[i]) {
            return execute[i];
        }
    }
    if (stage >= 3) {
        if (reg == dstRegs[5]) {
            return memory[0];
        }
        if (reg == dstRegs[6]) {
            return memory[1];
        }
    }

    return 0;
}

}; // namespace n16r

}; // namespace nbus

}; // namespace sysnp


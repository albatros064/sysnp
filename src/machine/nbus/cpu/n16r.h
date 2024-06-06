#ifndef SYSNP_N16R_H
#define SYSNP_N16R_H

#include "../nbus.h"
#include "cache.h"
#include "busunit.h"
#include <set>

namespace sysnp {

namespace nbus {

namespace n16r {

enum ExceptionType {
    ExceptInterrupt           = 000,
    // Vmem
    // Vmem
    // Vmem
    ExceptIllegalLoad         = 004,
    ExceptIllegalStore,
    ExceptIBus,
    ExceptDBus,
    ExceptSyscall,
    ExceptBreakpoint,
    ExceptReservedInstruction,
    ExceptInvalidInstruction,
    ExceptOverflow,
    ExceptNone                = 077
};

enum ExecuteOp {
    ExecuteNop,

    ExecuteAdd,
    ExecuteSubtract,
    ExecuteSetLessThan,
    ExecuteAnd,
    ExecuteOr,
    ExecuteNor,
    ExecuteXor,
    ExecuteLoadUpperImmediate,
    ExecuteLoadLowerImmediate,
    ExecuteLeftShift,
    ExecuteRightShift,
    ExecuteRightShiftArithmetic,
    ExecuteExchange,
    ExecutePickB
};
enum MemoryOp {
    MemoryNop,

    MemoryReadByte,
    MemoryReadWord,
    MemoryWriteByte,
    MemoryWriteWord
};
enum CommitOp {
    CommitNop,

    CommitWriteBack,
    CommitWrite,

    CommitJump,

    CommitDecideEQ,
    CommitDecideNE,
    CommitDecideGT,
    CommitDecideLE,
    CommitDecideLT,
    CommitDecideGE,

    CommitSyscall,
    CommitExceptionReturn,
    CommitExceptionReturnJump
};

class StageRegister {
    public:
        StageRegister();
        virtual ~StageRegister() {}

        void invalidate();

        uint32_t instructionPointer;
        uint32_t nextInstructionPointer;
        uint32_t altInstructionPointer;

        uint16_t word0;
        uint16_t word1;

        uint16_t regVals[6];
        uint8_t  srcRegs[6];
        uint8_t  dstRegs[4];

        ExecuteOp executeOp;
        MemoryOp  memoryOp;
        CommitOp  commitOp;

        bool executeCanOverflow;

        bool flushOnCommit;

        bool privileged;
        bool delayed;
        bool bubble;
        bool taken;
        bool exception;
        ExceptionType exceptionType;

        const static uint8_t emptyRegister = 255;
};

class N16R : public NBusDevice {
    public:
        virtual ~N16R() {}

        virtual void init(const libconfig::Setting&);
        virtual void postInit();

        virtual void clockUp();
        virtual void clockDown();

        virtual std::string command(std::stringstream&);

        // breakpoints
        virtual void breakpointClear();
        virtual void breakpointAdd(uint32_t);
        virtual void breakpointRemove(uint32_t);
        virtual bool breakpointHit();
    private:
        uint16_t registerFile[48];

        const static uint8_t causeRegister = 32;
        const static uint8_t statusRegister = 33;
        const static uint8_t ipcRegister = 42;
        const static uint8_t bvaRegister = 44;
        const static uint8_t epcRegister = 46;

        std::vector<StageRegister> stageRegisters;
        std::vector<StageRegister> stageRegistersOut;
        std::set<uint8_t> registerHazards;

        void fetchStage();
        void decodeStage();
        void executeStage();
        void memoryStage();
        void writeBackStage();

        void stageFlush(int);
        void stageShift();
        void stageClearOut();

        CacheController cacheController;

        BusUnit busUnit;

        uint32_t resetAddress;
        bool isPipelined;

        bool isKernel();
        bool hasInterrupts();

        void reset();

        // breakpoints
        uint32_t lastBreakpoint;
        bool breakpointWasHit;
        std::set<uint32_t> breakpoints;
};

}; // namespace n16r

}; // namespace nbus

}; // namespace sysnp

#endif

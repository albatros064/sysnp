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
    Interrupt           = 000,
    // Vmem
    // Vmem
    // Vmem
    IllegalLoad         = 004,
    IllegalStore,
    IBus,
    DBus,
    Syscall,
    Breakpoint,
    ReservedInstruction,
    InvalidInstruction,
    Overflow,
    ExceptionNone       = 076,
    ExceptionInhibit
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
    ExecuteExchange
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

    CommitDecideEQ,
    CommitDecideNE,
    CommitDecideGT,
    CommitDecideLE,
    CommitDecideLT,
    CommitDecideGE
};

class StageRegister {
    public:
        StageRegister();
        virtual ~StageRegister() {}

        void invalidate();

        uint32_t instructionPointer;
        uint32_t nextInstructionPointer;
        uint32_t altInstructionPointer;

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
        bool exception;

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
    private:
        uint16_t registerFile[48];

        std::vector<StageRegister> stageRegisters;
        std::set<uint8_t> registerHazards;

        void fetchStage();
        void decodeStage();
        void executeStage();
        void memoryStage();
        void writeBackStage();

        void stageFlush(int);
        void stageShift();

        CacheController cacheController;

        BusUnit busUnit;

        uint32_t resetAddress;
        bool isPipelined;

        bool isKernel();

        void reset();
};

}; // namespace n16r

}; // namespace nbus

}; // namespace sysnp

#endif

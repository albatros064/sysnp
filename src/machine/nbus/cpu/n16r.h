#ifndef SYSNP_N16R_H
#define SYSNP_N16R_H

#include "../nbus.h"
#include "busunit.h"
#include "memoryUnit.h"
#include <boost/circular_buffer.hpp>
#include <set>

namespace sysnp {

namespace nbus {

namespace n16r {

enum ExceptionType {
    ExceptInterrupt           = 000,
    ExceptTlbFault,
    ExceptProtFault,
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
    ExecuteAddHalf,
    ExecuteSubtract,
    ExecuteSubtractHalf,
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

    MemoryRead,
    MemoryWrite,
    MemoryReadByte,
    MemoryReadWord,
    MemoryReadDword,
    MemoryWriteByte,
    MemoryWriteWord,
    MemoryWriteDword
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

    CommitHalt,

    CommitLoadTlb,
    CommitExpireTlb,
    CommitFlushTlb,

    CommitSyscall,
    CommitExceptionReturn,
    CommitExceptionReturnJump
};
enum OperandHazard {
    OperandHazardNone,
    OperandHazardCurrent,
    OperandHazardNext
};

enum CanOverflow {
    CanNotOverflow,
    CanOverflow16,
    CanOverflow32
};

class StageRegister {
    public:
        StageRegister();
        virtual ~StageRegister() {}

        void invalidate();

        uint32_t instructionPointer;
        uint32_t nextInstructionPointer;
        uint32_t altInstructionPointer;

        uint16_t fetch  [2];
        uint16_t decode [5];
        uint16_t execute[5];
        uint16_t memory [2];
        uint8_t  srcRegs[5];
        uint8_t  dstRegs[7];

        uint32_t getQWord(int, uint16_t*);

        ExecuteOp executeOp;
        MemoryOp  memoryOp;
        CommitOp  commitOp;

        uint8_t memoryBytes;

        CanOverflow executeCanOverflow;

        bool flushOnCommit;

        bool privileged;
        bool privilegedInstruction;
        bool privilegedRead;
        bool privilegedWrite;
        bool delayed;
        bool bubble;
        bool taken;

        bool exception;
        ExceptionType exceptionType;
        uint32_t exceptionAddress;

        OperandHazard checkOperandHazard(uint8_t, uint8_t);
        uint16_t operandForward(uint8_t, uint8_t);

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
        const static uint8_t asidRegister = 38;
        const static uint8_t ipcRegister = 42;
        const static uint8_t bvaRegister = 44;
        const static uint8_t epcRegister = 46;

        uint32_t getRegisterDWord(int);
        void     setRegisterDWord(int, uint32_t);

        uint16_t getWord (int, uint16_t*);
        uint32_t getDWord(int, uint16_t*);

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

        MemoryUnit memoryUnit;
        BusUnit    busUnit;

        uint32_t resetAddress;
        bool isPipelined;

        bool halted;

        bool isKernel();
        bool hasInterrupts();

        void reset();

        // breakpoints
        uint32_t lastBreakpoint;
        bool breakpointWasHit;
        std::set<uint32_t> breakpoints;

        boost::circular_buffer<uint32_t> retiredAddresses;
        uint64_t clockCount;
        uint64_t retiredCount;
};

}; // namespace n16r

}; // namespace nbus

}; // namespace sysnp

#endif

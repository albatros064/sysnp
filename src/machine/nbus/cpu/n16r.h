#ifndef SYSNP_N16R_H
#define SYSNP_N16R_H

#include "../nbus.h"
#include "cache.h"
#include <set>

namespace sysnp {

namespace nbus {

namespace n16r {

enum BusPhase {
    Idle,
    LowRead,
    LowReadWait,
    HighRead,
    HighReadWait,
    Write,
    WriteWait
};

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

    CommitDecideEQ,
    CommitDecideNE,
    CommitDecideGT,
    CommitDecideLE,
    CommitDecideLT,
    CommitDecideGE
};

enum ExtendFunction {
    SignExtend, ZeroExtend, SignExtendWide, NoExtend
};

class BusUnit {
    public:
        void setBusInterface(std::shared_ptr<NBusInterface>);
        void reset();
        void clockUp();
        void clockDown();

        void addLowPriorityRead(uint32_t);
        void addHighPriorityRead(uint32_t);
        void addHighPriorityWrite(uint32_t, uint16_t, uint8_t);

        bool isReadReady();
        bool isHighReadPriority();
        uint16_t getReadData();
        uint32_t getReadAddress();

        uint8_t hasInterrupt();

    private:
        std::shared_ptr<NBusInterface> interface;

        BusPhase phase;

        bool readReady;
        bool readPriority;
        uint16_t readData;
        uint32_t readAddress;

        uint32_t addressInFlight;

        uint32_t lowPriorityAddress;
        uint32_t highPriorityAddress;
        uint16_t highPriorityData;
        uint8_t writeMode;
        bool hasLowPriority;
        bool hasHighPriority;
        bool hasHighPriorityWrite;

        uint8_t interruptState;
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

        ExtendFunction executeExtend;
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

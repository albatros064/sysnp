#ifndef SYSNP_N16R_H
#define SYSNP_N16R_H

#include "../nbus.h"

namespace sysnp {

namespace nbus {

namespace n16r {

enum ExecutionPhase {
    Fetch,
    FetchAdditional,
    FetchShielded,
    Execute,
    ExecuteAdditional,
    Commit,
    CommitAdditional,
    CommitShielded,
    CommitAdditionalShielded,
    Exception,
    ExceptionAdditional
};
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
enum InstructionFormat {
    R,  // register
    I,  // immediate
    M,  // memory
    E,  // memory (reg)
    J,  // jump
    B,  // branch
    NoFormat
};
enum AluFunction {
    Add, Sub, And, Or, Xor, Nor, LShift, RShift, RShiftA, Lui, Lli, NoFunction
};
enum BranchFunction {
    Equal,      // equal
    NotEqual,   // greater than or less than
    GreaterThan,// greater than
    NotGreater, // less than or equal
    LessThan,   // less than
    NotLess     // greater than or equal
};
enum ExtendFunction {
    Sign, Zero, SignAdditional, NoExtend
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

class ExecutionBuffer {
    public:
        bool isEmpty();
        bool isFull();
        uint8_t size();

        bool hasAddress(uint32_t);
        void addRequest(uint32_t);

        bool hasRequest();
        uint32_t getNextAddress();

        void checkBus(BusUnit&);
        uint16_t popTo(uint32_t);
        void push(uint32_t, uint16_t);

        void reset();
        void output(std::stringstream &);
    private:
        uint16_t buffer[4];
        uint32_t address[4];
        uint8_t head;
        uint8_t tail;

        uint32_t requestedAddress;
        bool pendingRequest;
};

class ArithmeticLogicUnit {
    public:
        void load(uint16_t, uint16_t);
        uint16_t calculate(AluFunction, ExtendFunction, bool);
        bool getOverflow();
        bool getNegative();
        bool getZero();
        bool branchTaken(BranchFunction);
    private:
        uint16_t a;
        uint16_t b;
        bool overflow;
        bool negative;
        bool zero;
};
class AddressCalculator {
    public:
        void loadLow(uint16_t, uint16_t);
        void loadHigh(uint16_t);
        uint32_t calculate();
    private:
        uint32_t base;
        uint32_t offset;
};

class DecoderUnit {
    public:
        int decode(uint16_t, uint32_t);
        bool isDouble;
        bool hasTrap;
        uint8_t opcode;
        uint8_t function;
        uint8_t dReg;
        uint8_t sReg;
        uint8_t rReg;
        uint16_t baseI;
        uint16_t extraI;
        uint32_t nextAddress;
        AluFunction aluFunction;
        BranchFunction branchFunction;
        ExtendFunction extend;
        InstructionFormat format;
};

class N16R : public NBusDevice {
    public:
        N16R();
        virtual ~N16R();

        virtual void init(const libconfig::Setting&);
        virtual void postInit();

        virtual void clockUp();
        virtual void clockDown();

        virtual std::string command(std::stringstream&);
    private:
        uint16_t registerFile   [16];
        uint16_t altRegisterFile[16];
        uint16_t sysRegisterFile[16];
        uint32_t instructionPointer;

        ExecutionPhase executionPhase;

        ExecutionBuffer executionBuffer;
        BusUnit busUnit;
        DecoderUnit decoder;
        ArithmeticLogicUnit alu;
        AddressCalculator addressCalculator;

        ExceptionType pendingException;

        uint32_t resetAddress;

        bool isKernel();

        void reset();
};

}; // namespace n16r

}; // namespace nbus

}; // namespace sysnp

#endif

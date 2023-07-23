#ifndef SYSNP_N16R_H
#define SYSNP_N16R_H

#include "../nbus.h"

namespace sysnp {

namespace nbus {

namespace n16r {

enum ExecutionPhase {
    Fetch,
    FetchAdditional,
    Execute,
    ExecuteAdditional,
    Commit,
    CommitAdditional
};
enum InstructionFormat {
    R, I, M, J, B
};
enum AluFunction {
    Add, Sub, And, Or, Xor, Nor, LShift, RShift, RShiftA, Lui, None
};

class ExecutionBuffer {
    public:
        bool isEmpty();
        bool isFull();

        bool hasAddress(uint32_t);
        void addRequest(uint32_t);

        bool hasRequest();
        uint32_t getNextAddress();

        uint16_t popTo(uint32_t);
        void push(uint16_t);

        void reset();
    private:
        uint16_t buffer[4];
        uint32_t address[4];
        uint8_t head;
        uint8_t tail;

        uint32_t requesedAddress;
        bool pendingRequest;
};

class BusUnit {
    public:
        void setBusInterface(std::shared_ptr<NBusInterface> interface);

        void addLowPriorityRead(uint32_t);
        void addHighPriorityRead(uint32_t);
        
    private:
};

class ArithmeticLogicUnit {
    public:
        void load(uint16_t, uint16_t);
        uint16_t calculate(uint8_t);
        bool getOverflow();
        bool getNegative();
        bool getZero();
    private:
        uint16_t a;
        uint16_t b;
        bool overflow;
        bool negative;
        bool zero;
};
class AddressCalculator {
    public:
        void load(uint32_t, uint16_t);
        uint32_t calculate();
};

class DecoderUnit {
    public:
        bool decode(uint16_t);
        bool isDouble;
        uint8_t opcode;
        uint8_t function;
        uint8_t dReg;
        uint8_t sReg;
        uint16_t baseI;
        uint16_t extraI;
        uint32_t nextAddress;
        AluFunction aluFunction;
        InstructionFormat format;
};

class CoreUnit {
    public:
        bool branchTaken;
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
        uint16_t registerFile   [8];
        uint16_t altRegisterFile[8];
        uint32_t instructionPointer;

        ExecutionPhase executionPhase;
        uint32_t busRequest;
        bool     busBusy;

        ExecutionBuffer executionBuffer;
        BusUnit busUnit;
        DecoderUnit decoder;
        ArithmeticLogicUnit alu;
        AddressCalculator adressCalculator;

        uint32_t resetAddress;

        void reset();
};

}; // namespace n16r

}; // namespace nbus

}; // namespace sysnp

#endif

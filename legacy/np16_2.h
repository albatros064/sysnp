#ifndef NP16_2_H
#define NP16_2_H

#include "cpu_base.h"

#include <stdint.h>

#define FRAME_POINTER       13
#define STACK_POINTER       14
#define RETURN_ADDRESS      15
#define INSTRUCTION_POINTER 16
#define FLAGS_REGISTER      17
#define EFLAGS_REGISTER     20
#define BVA_REGISTER        24
#define STATUS_REGISTER     25
#define CAUSE_REGISTER      26
#define EPC_REGISTER        27

#define FLAG_CARRY    1

#define EXCEPT_INT       0
//efine EXCEPT           1
//efine EXCEPT           2
//efine EXCEPT           3
//efine EXCEPT           4
//efine EXCEPT           5
#define EXCEPT_IBUS      6
#define EXCEPT_DBUS      7
#define EXCEPT_SYSCALL   8
#define EXCEPT_BREAK     9
#define EXCEPT_RESERVED 10
//efine ECXEPT          11
#define EXCEPT_OVERFLOW 12

class NP16_2 : public CPU_Base {
public:
	NP16_2(uint8_t *, uint64_t);
	virtual ~NP16_2();

	virtual uint64_t step(uint8_t &);
	virtual char *decode(uint64_t);

	virtual void fire_interrupt(uint8_t);

protected:
	virtual uint16_t instr_pointer_addr() { return INSTRUCTION_POINTER; }
	virtual uint16_t frame_pointer_addr() { return FRAME_POINTER; }
	virtual uint16_t stack_pointer_addr() { return STACK_POINTER; }

	void fire_exception(uint8_t, uint32_t);

	void rfe();
};

#endif

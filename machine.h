#ifndef MACHINE_H
#define MACHINE_H

#include <stdint.h>

class CPU;

class Machine {
public:
	Machine(uint32_t,char*);
	virtual ~Machine();

	void run();

protected:
	uint8_t *_memory;
	uint32_t _memory_size;
	
	uint16_t *_registers;
	uint16_t *_registers_aux;

	CPU *_cpu;
};

#endif


#ifndef MACHINE_H
#define MACHINE_H

#include <stdint.h>

#include "cpu_base.h"

class Machine {
public:
	Machine(CPU_Base *, uint8_t *, uint64_t);
	virtual ~Machine();

	void run();

	static uint8_t *allocate_memory(uint64_t, char *);

protected:
	uint8_t *_memory;
	uint64_t _memory_size;
	
	CPU_Base *_cpu;
};

#endif


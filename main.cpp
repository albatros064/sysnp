#include "cpu_base.h"
#include "machine.h"

#include <stdlib.h>
#include <stdint.h>

#include "np16_2.h"

int main(int argc, char* argv[]) {
	uint32_t memory_size = 16777216;
	if (argc > 2) {
		memory_size = atoi(argv[2]);
	}
	uint8_t *memory = Machine::allocate_memory(memory_size, argv[1]);

	CPU_Base *cpu = new NP16_2(memory, memory_size);

	Machine machine(cpu, memory, memory_size);
	machine.run();
	return 0;
}


#include <iostream>

#include "machine.h"

int main(int argc, char* argv[]) {
    sysnp::Machine machine;
    machine.load("hardware.conf");
    
    std::cout << "Done" << std::endl;
    /*
	uint8_t *memory = Machine::allocate_memory(memory_size, argv[1]);

	CPU_Base *cpu = new NP16_2(memory, memory_size);

	Machine machine(cpu, memory, memory_size);
	machine.run();
    */
	return 0;
}


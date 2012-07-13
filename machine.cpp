#include "cpu.h"
#include "machine.h"

#include <stdlib.h>
#include <stdio.h>

Machine::Machine(uint32_t memory_size, char *boot_file) {
	_memory = (uint8_t *) malloc(memory_size);
	if (!_memory)
		printf("Couldn't allocate %d bytes.\n", memory_size);
	_cpu = new CPU(_memory, _memory_size);

	_registers = _cpu->register_file();
	_registers_aux = _cpu->register_aux();

	FILE* file = 0;
	errno_t error = fopen_s(&file, boot_file, "r");
	if (!error) {
		int c;
		int m = 0;
		do {
			c = getc(file);
			_memory[m++] = (uint8_t) c;
		}
		while (c != EOF);
	}
	else {
		printf("Couldn't load boot file %s\n", boot_file);
	}
}

Machine::~Machine() {
	free(_memory);

	delete _cpu;
	_cpu = 0;
}

void Machine::run() {
	_cpu->reset();
	char buffer[16];
	do {
		uint16_t instruction = _cpu->step();

		printf("\n"
			"+-------------------------+--------------------------------------------------+\n"
			"|  Instruction:  0x%04x   |  PC: 0x%04x                                      |\n"
			"+-------------------------+--------------+-----------------------------------+\n"
			"|                                        |                                   |\n"
			"|  Registers:                            |  Memory:                          |\n"
			"|                                        |                                   |\n",
			instruction, _registers_aux[15] - 2);
		for (int i = 0; i < 16; i++) {
			uint16_t _mem = (_registers_aux[15] - 2) % 8;
			_mem = _registers_aux[15] - 2 - _mem + i * 8;
			printf(
				"|  %02d: 0x%04x %02d: 0x%04x     %02d: 0x%04x  |  0x%04x: %02x %02x %02x %02x %02x %02x %02x %02x  |\n",
				i, _registers[i],
				i +16, _registers[i + 16],
				i, _registers_aux[i],
				_mem,
				_memory[_mem],
				_memory[_mem + 1],
				_memory[_mem + 2],
				_memory[_mem + 3],
				_memory[_mem + 4],
				_memory[_mem + 5],
				_memory[_mem + 6],
				_memory[_mem + 7]
			);
		}
		printf(
			"|                                        |                                   |\n"
			"+----------------------------------------+-----------------------------------+\n"
		);
		gets(buffer);
		if (buffer[0] == 'r') {
			_cpu->reset();
		}
	}
	while (buffer[0] != 'q');
}


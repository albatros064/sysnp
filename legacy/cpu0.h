#ifndef CPU_H
#define CPU_H

#include <stdint.h>

#define EXCEPTION_ADDRESS 0x2

#define HRDWARE_CAUSE 0x0
#define SYSCALL_CAUSE 0x1

class CPU {
public:
	CPU(uint8_t *memory_file, uint32_t memory_size);
	~CPU() {}

	void reset() { _register_aux[15] = 0; }

	uint16_t  step();
	uint32_t *register_file() { return _register_file; }
	uint32_t *register_aux()  { return _register_aux;  }

protected:
	uint8_t *_memory_file;
	uint32_t _memory_size;

	/* */
#define _acc _register_file[2]
	/* */
	uint32_t _register_file[32];

	/* */
#define _status _register_aux[0]
#define _cause _register_aux[1]
#define _delay_slot _register_aux[8]
#define _jump_pc _register_aux[9]
#define _exception_program_counter _register_aux[14]
#define _program_counter _register_aux[15]
	/* */
	uint32_t _register_aux[16];

	void execute_i_instruction     (uint8_t,uint8_t,uint8_t);
	void execute_r_0_instruction   (uint8_t,uint8_t,uint8_t);
	void execute_r_5_instruction   (uint8_t,uint8_t,uint8_t);
	void execute_r_6_instruction   (uint8_t,uint8_t,uint8_t);
	void execute_r_7_instruction   (uint8_t,uint8_t,uint8_t);
	void execute_h_instruction     (uint8_t,uint8_t,uint8_t);
	void execute_h_high_instruction(uint8_t,uint8_t,uint8_t);
	void execute_h_low_instruction (uint8_t,uint8_t,uint8_t);

	void execute_privelaged_instruction(uint8_t,uint8_t);

	void execute_syscall();

	void execute_no_op();

	void advance_program_counter();
};

#endif


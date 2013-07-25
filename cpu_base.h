#ifndef CPU_BASE_H
#define CPU_BASE_H

#include <stdint.h>

class CPU_Base {
public:
	CPU_Base(uint8_t, uint8_t, uint8_t, uint16_t, uint16_t, uint64_t, uint8_t *);
	virtual ~CPU_Base();

	virtual uint64_t step(uint8_t &) = 0;
	void reset() { _register_file[instr_pointer_addr()] = 0; }

	virtual char *decode(uint64_t) = 0;

	virtual void fire_interrupt(uint8_t) = 0;

	uint64_t instr_pointer() { return _register_file[instr_pointer_addr()]; }
	uint64_t frame_pointer() { return _register_file[frame_pointer_addr()]; }
	uint64_t stack_pointer() { return _register_file[stack_pointer_addr()]; }

	uint8_t data_bits    () { return _data_bits    ; }
	uint8_t logical_bits () { return _laddress_bits; }
	uint8_t physical_bits() { return _paddress_bits; }

	uint64_t *register_file(uint16_t &c) { c = _register_count;     return _register_file; }
	uint64_t *register_aux (uint16_t &c) { c = _register_count_aux; return _register_file + _register_count; }
protected:
	uint8_t _data_bits;
	uint8_t _laddress_bits;
	uint8_t _paddress_bits;

	uint64_t  _memory_size;
	uint8_t  *_memory_file;

	uint16_t  _register_count;
	uint16_t  _register_count_aux;
	uint64_t *_register_file;

	virtual uint16_t instr_pointer_addr() = 0;
	virtual uint16_t frame_pointer_addr() = 0;
	virtual uint16_t stack_pointer_addr() = 0;
};

#endif
